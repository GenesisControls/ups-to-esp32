/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "out_modbus.h"
#include "out_common.h"
#include "ups_hid.h"
#include "board.h"
#include "mbcontroller.h"
#include "driver/uart.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "out_mb";

/* Autobaud does not exist yet (its own milestone). Until it does, a board set
 * to "auto" has to pick something rather than sit mute on the trunk, and 9600
 * is the rate a BMS integrator reaches for first. Logged loudly, because a
 * silent wrong guess is worse than a noisy one. */
#define BAUD_FALLBACK   9600

/* THE ONE WRITABLE ADDRESS. Coil 100: start a quick battery self-test.
 *
 * Deliberately far from the mirrored discretes (0-10) so it can never be hit
 * by a master sweeping the status block, and in a range the other track's map
 * does not define. PROVISIONAL until they allocate it properly -- a command
 * address belongs in the shared contract, not in whichever implementation
 * wrote it first.
 *
 * Write 1 to start. It self-clears, so it is a trigger and not a state; a
 * master that leaves it set does not re-fire. */
#define CMD_COIL_ADDR   100
/* A stuck or misconfigured master must not be able to cycle the battery. A
 * quick test is seconds of load on battery; one every half hour is generous
 * for any real use and useless as a way to flatten a pack. */
#define CMD_MIN_GAP_US  (30LL * 60 * 1000000)
#define REFRESH_MS      1000

/* The register image the stack serves from. esp-modbus answers reads straight
 * out of these, so they are refreshed from out_common rather than built per
 * request -- a Modbus master polling hard must never be able to make us poll
 * the UPS hard. */
static uint16_t s_ireg[OUT_REG_N];
static uint16_t s_hreg[OUT_REG_N];          /* the FC3 mirror of the same data */
static uint8_t  s_disc[(OUT_DISCRETE_N + 7) / 8];
static uint8_t  s_coil[(OUT_DISCRETE_N + 7) / 8];

/* A one-minute sliding window, in six ten-second buckets: enough to show a
 * rate without keeping timestamps for every frame. */
#define RATE_BUCKETS    6
#define RATE_BUCKET_US  (10LL * 1000000)
static uint32_t s_bucket[RATE_BUCKETS];
static int64_t  s_bucket_at_us;
static int      s_bucket_i;

static uint8_t  s_cmd;          /* the command coil's one byte */
static int64_t  s_cmd_last_us;
static out_modbus_stat_t s_stat;
static SemaphoreHandle_t s_stat_lock;
static void *s_handler;

/* The contiguous blocks of the address map, with the reserved ranges left
 * OUT. This is how "a planned address answers exception 02" is enforced: an
 * address inside no registered area is one the stack rejects, so 12-15 and
 * 19-23 are genuinely absent rather than reading as a value. Registering
 * 0..23 as one block and filling the gaps with 65535 would erase the
 * distinction between "this UPS does not report it" and "this adapter does
 * not have it", which a commissioning engineer needs. */
typedef struct { uint16_t first, count; } block_t;
static const block_t REG_BLOCKS[] = {
    { 0,  12 },     /* 0-11  : the UPS. 0-7 are live in Niagara today */
    { 16, 3  },     /* 16-18 : the adapter */
};

static void refresh(void)
{
    out_points_t p;
    out_common_get(&p);

    for (int i = 0; i < OUT_REG_N; i++) {
        /* A register outside every block is never served, so what sits here
         * cannot be read; zero is simply tidier than stale data. */
        s_ireg[i] = p.reg_live[i] ? p.reg[i] : 0;
        s_hreg[i] = s_ireg[i];
    }
    memset(s_disc, 0, sizeof(s_disc));
    for (int i = 0; i < OUT_DISCRETE_N; i++) {
        if (p.discrete[i]) s_disc[i / 8] |= (uint8_t)(1u << (i % 8));
    }
    memcpy(s_coil, s_disc, sizeof(s_coil));
}

/* One task: refresh the image, and watch what the trunk does to it.
 *
 * READ-ONLY, AND A CAVEAT WORTH READING. The other track's map says every
 * write is refused with exception 02, enforced in the datastore. pymodbus can
 * do that; esp-modbus 1.0 cannot -- a registered holding or coil area is
 * writable, and the stack has no read-only flag and no hook that runs before
 * the write lands. The choice is therefore between:
 *
 *   (a) not registering the holding and coil areas, so writes fail -- but so
 *       do FC3 and FC1 READS, which the map explicitly requires ("mirrored
 *       as holding registers, so either function code works from Niagara");
 *   (b) registering them, and undoing any write immediately.
 *
 * (b), because breaking reads on half the function codes to make writes fail
 * correctly would be the worse trade for a site: a Niagara point sheet
 * configured against FC3 would simply stop working.
 *
 * So a write is accepted at the protocol level and then has no effect -- the
 * next refresh, within a second, puts the value back, and this counts it and
 * shows it on the status page. It does NOT produce exception 02. That is a
 * genuine deviation from the contract and belongs on the other track's desk,
 * not buried here. */
/* Coil 100 went high. Everything that can refuse this, refuses it here.
 *
 * Reversing "read-only on the bus" was Tobias's call on 2026-11-22; the
 * reasons it was the rule are unchanged, so the guards are the compensation:
 * the board owner opts in, the UPS must say it can self-test, and the rate
 * limit means the worst a hostile or broken master achieves is one quick
 * test every half hour. */
static void cmd_check(void)
{
    if (!(s_cmd & 1u)) return;
    s_cmd = 0;                                  /* a trigger, not a state */

    if (!out_common_cmd_enabled()) {
        ESP_LOGW(TAG, "self-test requested from the bus, but commands are off on this board");
        return;
    }
    ups_data_t d;
    ups_hid_get(&d);
    if (!d.attached) {
        ESP_LOGW(TAG, "self-test requested, but no UPS is attached");
        return;
    }
    if (!(d.caps & UPS_CAP_TEST)) {
        /* From the UPS's own report descriptor, never from its model name. */
        ESP_LOGW(TAG, "self-test requested, but this UPS does not report that it can");
        return;
    }
    int64_t now = esp_timer_get_time();
    if (s_cmd_last_us && (now - s_cmd_last_us) < CMD_MIN_GAP_US) {
        ESP_LOGW(TAG, "self-test requested %llds after the last one; ignored (minimum %lld min)",
                 (long long)((now - s_cmd_last_us) / 1000000),
                 (long long)(CMD_MIN_GAP_US / 60000000));
        return;
    }
    s_cmd_last_us = now;
    esp_err_t e = ups_hid_command(UPS_CMD_TEST_QUICK, 0);
    ESP_LOGW(TAG, "QUICK BATTERY SELF-TEST started from the bus: %s. "
                  "The load is on battery while it runs.", esp_err_to_name(e));
    xSemaphoreTake(s_stat_lock, portMAX_DELAY);
    s_stat.tests_started++;
    xSemaphoreGive(s_stat_lock);
}

static void mb_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(REFRESH_MS);
    TickType_t next = xTaskGetTickCount();
    for (;;) {
        /* Drain whatever the trunk has done since the last pass. */
        mb_event_group_t ev = mbc_slave_check_event(
            MB_EVENT_HOLDING_REG_WR | MB_EVENT_HOLDING_REG_RD |
            MB_EVENT_INPUT_REG_RD   | MB_EVENT_COILS_WR       |
            MB_EVENT_COILS_RD       | MB_EVENT_DISCRETE_RD);

        if (ev) {
            xSemaphoreTake(s_stat_lock, portMAX_DELAY);
            s_stat.requests++;
            s_bucket[s_bucket_i]++;
            s_stat.last_request_us = esp_timer_get_time();
            if (ev & MB_EVENT_INPUT_REG_RD)   s_stat.rd_input++;
            if (ev & MB_EVENT_HOLDING_REG_RD) s_stat.rd_holding++;
            if (ev & MB_EVENT_DISCRETE_RD)    s_stat.rd_discrete++;
            if (ev & MB_EVENT_COILS_RD)       s_stat.rd_coil++;
            if (ev & (MB_EVENT_HOLDING_REG_WR | MB_EVENT_COILS_WR)) {
                s_stat.writes_refused++;
            }
            xSemaphoreGive(s_stat_lock);
            if (ev & MB_EVENT_COILS_WR) cmd_check();
            if (ev & (MB_EVENT_HOLDING_REG_WR | MB_EVENT_COILS_WR)) {
                /* The status coils and every register are read-only; only the
                 * command coil above means anything, and cmd_check() has
                 * already consumed it. */
                refresh();                  /* put the status values back */
            }
        }

        /* Roll the rate window, and sample the collision flag. The flag is
         * sticky until read, so reading it here both counts and clears it. */
        int64_t now_us = esp_timer_get_time();
        if (now_us - s_bucket_at_us >= RATE_BUCKET_US) {
            s_bucket_at_us = now_us;
            s_bucket_i = (s_bucket_i + 1) % RATE_BUCKETS;
            uint32_t sum = 0;
            xSemaphoreTake(s_stat_lock, portMAX_DELAY);
            s_bucket[s_bucket_i] = 0;               /* reuse the oldest */
            for (int i = 0; i < RATE_BUCKETS; i++) sum += s_bucket[i];
            s_stat.per_min = sum;
            xSemaphoreGive(s_stat_lock);
        }
        bool collided = false;
        if (uart_get_collision_flag(BOARD_RS485_UART, &collided) == ESP_OK && collided) {
            xSemaphoreTake(s_stat_lock, portMAX_DELAY);
            s_stat.collisions++;
            xSemaphoreGive(s_stat_lock);
            ESP_LOGW(TAG, "RS485 collision: another device keyed the line while we "
                          "were transmitting -- check for a duplicate address");
        }

        if (xTaskGetTickCount() - next >= period) {
            next = xTaskGetTickCount();
            refresh();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* The register areas and the stack start. Identical for RTU and TCP -- the
 * point set does not care what carries it -- so this is shared rather than
 * written twice and left to drift. */
static esp_err_t areas_and_start(void)
{
    esp_err_t err;

    refresh();

    /* Registering the blocks, gaps and all. Input and holding carry the same
     * data so FC4 and FC3 agree; discretes and coils likewise for FC2/FC1. */
    for (size_t b = 0; b < sizeof(REG_BLOCKS) / sizeof(REG_BLOCKS[0]); b++) {
        const block_t *blk = &REG_BLOCKS[b];
        mb_register_area_descriptor_t a = {
            .start_offset = blk->first,
            .type         = MB_PARAM_INPUT,
            .address      = &s_ireg[blk->first],
            .size         = (size_t)blk->count * sizeof(uint16_t),
        };
        if ((err = mbc_slave_set_descriptor(a)) != ESP_OK) goto fail;
        a.type    = MB_PARAM_HOLDING;
        a.address = &s_hreg[blk->first];
        if ((err = mbc_slave_set_descriptor(a)) != ESP_OK) goto fail;
    }
    {
        mb_register_area_descriptor_t d = {
            .start_offset = 0,
            .type         = MB_PARAM_DISCRETE,
            .address      = s_disc,
            .size         = sizeof(s_disc),
        };
        if ((err = mbc_slave_set_descriptor(d)) != ESP_OK) goto fail;
        d.type    = MB_PARAM_COIL;
        d.address = s_coil;
        if ((err = mbc_slave_set_descriptor(d)) != ESP_OK) goto fail;

        /* The command coil, in its own area well away from the status block.
         * Registered whether or not commands are enabled, so a master gets a
         * clean "0" rather than an exception from a board that merely has the
         * feature switched off -- cmd_check() is what refuses the action. */
        mb_register_area_descriptor_t cm = {
            .start_offset = CMD_COIL_ADDR,
            .type         = MB_PARAM_COIL,
            .address      = &s_cmd,
            .size         = sizeof(s_cmd),
        };
        if ((err = mbc_slave_set_descriptor(cm)) != ESP_OK) goto fail;
    }

    if ((err = mbc_slave_start()) != ESP_OK) {
        ESP_LOGE(TAG, "slave start: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;

fail:
    ESP_LOGE(TAG, "register area: %s", esp_err_to_name(err));
    return err;
}

/* Modbus TCP. The stack's TCP slave is already compiled in
 * (CONFIG_FMB_COMM_MODE_TCP_EN defaults on), so this costs no new dependency.
 *
 * Bound to the STATION interface on purpose. The setup access point is open
 * and unauthenticated, and a listener on every interface would serve the
 * point set there too; a BMS reaches this board over the network it joined,
 * which is the only place this data belongs. */
static esp_err_t start_tcp(int addr)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) {
        ESP_LOGE(TAG, "Modbus TCP needs the Wi-Fi station interface, which is not up");
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = mbc_slave_init(MB_PORT_TCP_SLAVE, &s_handler);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "slave init (TCP): %s", esp_err_to_name(err));
        return err;
    }
    /* mb_communication_info_t is a UNION: these are the TCP arm's fields, and
     * the serial arm's must be left alone. ip_addr NULL accepts any client --
     * the brief's allowed-client list is a later setting, and until it exists
     * the honest position is that anything on that LAN can read this. */
    const int port = out_common_mb_tcp_port();
    mb_communication_info_t comm = {
        .ip_mode      = MB_MODE_TCP,
        .slave_uid    = (uint8_t)addr,
        .ip_port      = (uint16_t)port,
        .ip_addr_type = MB_IPV4,
        .ip_addr      = NULL,
        .ip_netif_ptr = netif,
    };
    if ((err = mbc_slave_setup(&comm)) != ESP_OK) {
        ESP_LOGE(TAG, "slave setup (TCP): %s", esp_err_to_name(err));
        return err;
    }
    if ((err = areas_and_start()) != ESP_OK) return err;

    xSemaphoreTake(s_stat_lock, portMAX_DELAY);
    s_stat.running = true;
    s_stat.address = addr;
    s_stat.baud    = 0;                 /* a socket has no baud rate */
    s_stat.format  = "TCP";
    s_stat.port    = port;
    xSemaphoreGive(s_stat_lock);

    if (xTaskCreate(mb_task, "out_mb", 3072, NULL, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Modbus TCP slave up: unit %d on port %d, up to %d masters",
             addr, port, CONFIG_FMB_TCP_PORT_MAX_CONN);
    ESP_LOGI(TAG, "bus commands %s (coil %d starts a quick self-test)",
             out_common_cmd_enabled() ? "ENABLED" : "off", CMD_COIL_ADDR);
    return ESP_OK;
}

esp_err_t out_modbus_start(void)
{
    const out_proto_t proto = out_common_proto();
    if (proto != OUT_PROTO_MODBUS_RTU && proto != OUT_PROTO_MODBUS_TCP) {
        return ESP_OK;                      /* not ours: cost the board nothing */
    }

    s_stat_lock = xSemaphoreCreateMutex();
    if (!s_stat_lock) return ESP_ERR_NO_MEM;

    const int addr = out_common_address();

    if (proto == OUT_PROTO_MODBUS_TCP) return start_tcp(addr);

#if !BOARD_HAS_RS485
    ESP_LOGE(TAG, "Modbus RTU selected, but this board has no RS485 transceiver");
    return ESP_ERR_NOT_SUPPORTED;
#else
    int baud = out_common_baud();
    if (baud == OUT_BAUD_AUTO) {
        baud = BAUD_FALLBACK;
        ESP_LOGW(TAG, "baud is set to auto but autobaud is not built yet: using %d", baud);
    }
    const out_fmt_t fmt = out_common_format();
    const uart_parity_t par = (fmt == OUT_FMT_8E1) ? UART_PARITY_EVEN
                            : (fmt == OUT_FMT_8O1) ? UART_PARITY_ODD
                                                   : UART_PARITY_DISABLE;

    esp_err_t err = mbc_slave_init(MB_PORT_SERIAL_SLAVE, &s_handler);
    if (err != ESP_OK) { ESP_LOGE(TAG, "slave init: %s", esp_err_to_name(err)); return err; }

    mb_communication_info_t comm = {
        .mode       = MB_MODE_RTU,
        .slave_addr = (uint8_t)addr,
        .port       = BOARD_RS485_UART,
        .baudrate   = (uint32_t)baud,
        .parity     = par,
    };
    if ((err = mbc_slave_setup(&comm)) != ESP_OK) {
        ESP_LOGE(TAG, "slave setup: %s", esp_err_to_name(err));
        return err;
    }

    if ((err = areas_and_start()) != ESP_OK) return err;

    /* After the stack has the UART. The driver-enable is the peripheral's own
     * RTS in half-duplex mode -- see board.h; keying it in software is a race
     * that clips frames at the wire with nothing on the board looking wrong. */
    if ((err = uart_set_pin(BOARD_RS485_UART, BOARD_PIN_RS485_TX, BOARD_PIN_RS485_RX,
                            BOARD_PIN_RS485_DE, UART_PIN_NO_CHANGE)) != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin: %s", esp_err_to_name(err));
        return err;
    }
    if ((err = uart_set_mode(BOARD_RS485_UART, UART_MODE_RS485_HALF_DUPLEX)) != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_mode RS485: %s", esp_err_to_name(err));
        return err;
    }
    /* Stop bits are not in mb_communication_info_t, so esp-modbus leaves
     * whatever the UART defaults to. Set them explicitly after the stack has
     * the port, or an 8N2 master sees frame errors that look exactly like a
     * wrong baud rate. */
    if ((err = uart_set_stop_bits(BOARD_RS485_UART,
                                  fmt == OUT_FMT_8N2 ? UART_STOP_BITS_2
                                                     : UART_STOP_BITS_1)) != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_stop_bits: %s", esp_err_to_name(err));
        return err;
    }

    xSemaphoreTake(s_stat_lock, portMAX_DELAY);
    s_stat.running = true;
    s_stat.address = addr;
    s_stat.baud    = baud;
    s_stat.format  = out_common_format_name(fmt);
    xSemaphoreGive(s_stat_lock);

    if (xTaskCreate(mb_task, "out_mb", 3072, NULL, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Modbus RTU slave up: address %d, %d baud %s, "
                  "TX %d RX %d DE %d (hardware RTS)",
             addr, baud, out_common_format_name(fmt),
             BOARD_PIN_RS485_TX, BOARD_PIN_RS485_RX, BOARD_PIN_RS485_DE);
    ESP_LOGI(TAG, "bus commands %s (coil %d starts a quick self-test)",
             out_common_cmd_enabled() ? "ENABLED" : "off", CMD_COIL_ADDR);
    return ESP_OK;
#endif /* BOARD_HAS_RS485 */
}

void out_modbus_get(out_modbus_stat_t *out)
{
    if (!s_stat_lock) { memset(out, 0, sizeof(*out)); return; }
    xSemaphoreTake(s_stat_lock, portMAX_DELAY);
    *out = s_stat;
    xSemaphoreGive(s_stat_lock);
}
