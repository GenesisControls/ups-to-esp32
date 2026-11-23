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

/* 2.x asks for a response timeout per instance. This is how long the stack
 * waits on its own internals, not how long a master waits for us; the default
 * is fine and this is here to be explicit rather than to tune anything. */
#define RESPONSE_TOUT_MS 1000
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
/* PER-INSTANCE state. Two Modbus slaves can run at once -- RTU on the trunk
 * and TCP on the network -- and everything that describes one of them has to
 * be one of these rather than a file-level static.
 *
 * The register image above is deliberately NOT in here. Both instances serve
 * from the same arrays, which is the whole point: one shared point set, so
 * RTU and TCP cannot answer differently in the same second. */
typedef struct {
    void             *ctx;          /* the 2.x instance handle */
    const char       *what;         /* "RTU" / "TCP", for logs */
    bool              is_rtu;       /* collisions are a UART concept only */
    out_modbus_stat_t stat;
    SemaphoreHandle_t lock;
    uint32_t          bucket[RATE_BUCKETS];
    int64_t           bucket_at_us;
    int               bucket_i;
} inst_t;

static inst_t s_rtu = { .what = "RTU", .is_rtu = true  };
static inst_t s_tcp = { .what = "TCP", .is_rtu = false };

/* The command coil and its rate limit are DEVICE-WIDE, not per instance. One
 * self-test per thirty minutes is a statement about the battery, not about a
 * transport, and keeping them per instance would let a master on the trunk
 * and a client over TCP each start one inside the same window. */
static uint8_t  s_cmd;          /* the command coil's one byte */
static int64_t  s_cmd_last_us;

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

/* One task per running instance: watch what its master does to the image.
 *
 * READ-ONLY, AND HOW IT IS ENFORCED. The other track's map says every write
 * is refused with an exception, enforced in the datastore. Under esp-modbus
 * 1.x that was not possible -- a registered holding or coil area was
 * writable, with no read-only flag and no hook before the write landed -- so
 * this component used to register them writable and put the real value back
 * on the next refresh, which left a window where a master could read what it
 * had just written and produced no exception at all. That was a real
 * deviation from the contract and was relayed to the other track as one.
 *
 * It is gone. 2.x takes an access type per area, so the status registers and
 * coils are MB_ACCESS_RO and the stack itself refuses the write. Only the
 * command coil is writable, because cmd_check() has to see what was written
 * in order to refuse it on its own terms. */
/* Coil 100 went high. Everything that can refuse this, refuses it here.
 *
 * Reversing "read-only on the bus" was Tobias's call on 2026-11-22; the
 * reasons it was the rule are unchanged, so the guards are the compensation:
 * the board owner opts in, the UPS must say it can self-test, and the rate
 * limit means the worst a hostile or broken master achieves is one quick
 * test every half hour. */
static void cmd_check(inst_t *in)
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
    xSemaphoreTake(in->lock, portMAX_DELAY);
    in->stat.tests_started++;
    xSemaphoreGive(in->lock);
}

static void mb_task(void *arg)
{
    inst_t *in = (inst_t *)arg;
    const TickType_t period = pdMS_TO_TICKS(REFRESH_MS);
    TickType_t next = xTaskGetTickCount();
    for (;;) {
        /* Drain whatever this master has done since the last pass. */
        mb_event_group_t ev = mbc_slave_check_event(in->ctx,
            MB_EVENT_HOLDING_REG_WR | MB_EVENT_HOLDING_REG_RD |
            MB_EVENT_INPUT_REG_RD   | MB_EVENT_COILS_WR       |
            MB_EVENT_COILS_RD       | MB_EVENT_DISCRETE_RD);

        if (ev) {
            xSemaphoreTake(in->lock, portMAX_DELAY);
            in->stat.requests++;
            in->bucket[in->bucket_i]++;
            in->stat.last_request_us = esp_timer_get_time();
            if (ev & MB_EVENT_INPUT_REG_RD)   in->stat.rd_input++;
            if (ev & MB_EVENT_HOLDING_REG_RD) in->stat.rd_holding++;
            if (ev & MB_EVENT_DISCRETE_RD)    in->stat.rd_discrete++;
            if (ev & MB_EVENT_COILS_RD)       in->stat.rd_coil++;
            if (ev & (MB_EVENT_HOLDING_REG_WR | MB_EVENT_COILS_WR)) {
                in->stat.writes_refused++;
            }
            xSemaphoreGive(in->lock);
            if (ev & MB_EVENT_COILS_WR) cmd_check(in);
            /* No put-the-values-back pass: the status areas are MB_ACCESS_RO,
             * so a write to one never reaches our buffer.
             *
             * UNVERIFIED, and worth checking against a real master: whether
             * 2.x still raises a _WR event for a write it refused itself. If
             * it refuses before signalling, writes_refused will read 0 while
             * writes are in fact being correctly rejected -- a worse counter
             * for better behaviour on the wire, which is the right way
             * round. The exception the master sees is what the acceptance
             * test checks, not this. */
        }

        /* Roll the rate window. */
        int64_t now_us = esp_timer_get_time();
        if (now_us - in->bucket_at_us >= RATE_BUCKET_US) {
            in->bucket_at_us = now_us;
            in->bucket_i = (in->bucket_i + 1) % RATE_BUCKETS;
            uint32_t sum = 0;
            xSemaphoreTake(in->lock, portMAX_DELAY);
            in->bucket[in->bucket_i] = 0;           /* reuse the oldest */
            for (int i = 0; i < RATE_BUCKETS; i++) sum += in->bucket[i];
            in->stat.per_min = sum;
            xSemaphoreGive(in->lock);
        }

        /* Collisions are an RS485 fact: two devices keying one pair of wires.
         * There is no such thing on a socket, so the TCP instance does not
         * look -- and must not, since the flag belongs to a UART it may not
         * even be sharing. The flag is sticky until read, so reading it here
         * both counts and clears it. */
        if (in->is_rtu) {
            bool collided = false;
            if (uart_get_collision_flag(BOARD_RS485_UART, &collided) == ESP_OK && collided) {
                xSemaphoreTake(in->lock, portMAX_DELAY);
                in->stat.collisions++;
                xSemaphoreGive(in->lock);
                ESP_LOGW(TAG, "RS485 collision: another device keyed the line while we "
                              "were transmitting -- check for a duplicate address");
            }
        }

        /* Only one instance needs to rebuild the shared image; the RTU task
         * owns it when both are up, so the work is not done twice. refresh()
         * is pure, so this is about waste rather than correctness. */
        if (xTaskGetTickCount() - next >= period) {
            next = xTaskGetTickCount();
            if (in->is_rtu || !s_rtu.ctx) refresh();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* The register areas and the stack start. Identical for RTU and TCP -- the
 * point set does not care what carries it -- so this is shared rather than
 * written twice and left to drift. */
static esp_err_t areas_and_start(void *ctx)
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
            /* MB_ACCESS_RO is new in 2.x and it is what the data map always
             * wanted: the STACK refuses a write and answers an exception,
             * instead of the 1.x arrangement where a write landed in our
             * buffer and refresh() put the real value back a moment later.
             * That window is gone, and so is the deviation it forced us to
             * document for the other track. */
            .access       = MB_ACCESS_RO,
            .address      = &s_ireg[blk->first],
            .size         = (size_t)blk->count * sizeof(uint16_t),
        };
        if ((err = mbc_slave_set_descriptor(ctx, a)) != ESP_OK) goto fail;
        a.type    = MB_PARAM_HOLDING;
        a.address = &s_hreg[blk->first];
        if ((err = mbc_slave_set_descriptor(ctx, a)) != ESP_OK) goto fail;
    }
    {
        mb_register_area_descriptor_t d = {
            .start_offset = 0,
            .type         = MB_PARAM_DISCRETE,
            .access       = MB_ACCESS_RO,
            .address      = s_disc,
            .size         = sizeof(s_disc),
        };
        if ((err = mbc_slave_set_descriptor(ctx, d)) != ESP_OK) goto fail;
        d.type    = MB_PARAM_COIL;
        d.address = s_coil;
        if ((err = mbc_slave_set_descriptor(ctx, d)) != ESP_OK) goto fail;

        /* The command coil, in its own area well away from the status block.
         * Registered whether or not commands are enabled, so a master gets a
         * clean "0" rather than an exception from a board that merely has the
         * feature switched off -- cmd_check() is what refuses the action. */
        mb_register_area_descriptor_t cm = {
            .start_offset = CMD_COIL_ADDR,
            .type         = MB_PARAM_COIL,
            /* The ONE writable thing on this device, and it is writable only
             * so cmd_check() can read what was written and then refuse it on
             * its own terms. Everything else above is MB_ACCESS_RO. */
            .access       = MB_ACCESS_RW,
            .address      = &s_cmd,
            .size         = sizeof(s_cmd),
        };
        if ((err = mbc_slave_set_descriptor(ctx, cm)) != ESP_OK) goto fail;
    }

    if ((err = mbc_slave_start(ctx)) != ESP_OK) {
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
    inst_t *in = &s_tcp;

    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) {
        ESP_LOGE(TAG, "Modbus TCP needs the Wi-Fi station interface, which is not up");
        return ESP_ERR_INVALID_STATE;
    }
    in->lock = xSemaphoreCreateMutex();
    if (!in->lock) return ESP_ERR_NO_MEM;

    /* ip_addr_table NULL accepts any client -- the brief's allowed-client
     * list is a later setting, and until it exists the honest position is
     * that anything on that LAN can read this. */
    const int port = out_common_mb_tcp_port();
    mb_communication_info_t comm = {
        .tcp_opts = {
            .mode             = MB_TCP,
            .port             = (uint16_t)port,
            .uid              = (uint8_t)addr,
            .addr_type        = MB_IPV4,
            .ip_addr_table    = NULL,
            .ip_netif_ptr     = netif,
            .response_tout_ms = RESPONSE_TOUT_MS,
        },
    };
    esp_err_t err = mbc_slave_create_tcp(&comm, &in->ctx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "slave create (TCP): %s", esp_err_to_name(err));
        return err;
    }
    if ((err = areas_and_start(in->ctx)) != ESP_OK) return err;

    xSemaphoreTake(in->lock, portMAX_DELAY);
    in->stat.running = true;
    in->stat.address = addr;
    in->stat.baud    = 0;               /* a socket has no baud rate */
    in->stat.format  = "TCP";
    in->stat.port    = port;
    xSemaphoreGive(in->lock);

    if (xTaskCreate(mb_task, "out_mb_tcp", 3072, in, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Modbus TCP slave up: unit %d on port %d, up to %d clients",
             addr, port, CONFIG_FMB_TCP_PORT_MAX_CONN);
    return ESP_OK;
}

#if BOARD_HAS_RS485
/* Modbus RTU on the trunk. */
static esp_err_t start_rtu(int addr)
{
    inst_t *in = &s_rtu;

    in->lock = xSemaphoreCreateMutex();
    if (!in->lock) return ESP_ERR_NO_MEM;

    int baud = out_common_baud();
    if (baud == OUT_BAUD_AUTO) {
        baud = BAUD_FALLBACK;
        ESP_LOGW(TAG, "baud is set to auto but autobaud is not built yet: using %d", baud);
    }
    const out_fmt_t fmt = out_common_format();
    const uart_parity_t par = (fmt == OUT_FMT_8E1) ? UART_PARITY_EVEN
                            : (fmt == OUT_FMT_8O1) ? UART_PARITY_ODD
                                                   : UART_PARITY_DISABLE;

    /* 2.x carries the whole character format in the config, including stop
     * bits -- which 1.x did not, and which forced a uart_set_stop_bits() call
     * after the stack had the port. That workaround is gone. */
    mb_communication_info_t comm = {
        .ser_opts = {
            .mode             = MB_RTU,
            .port             = BOARD_RS485_UART,
            .uid              = (uint8_t)addr,
            .baudrate         = (uint32_t)baud,
            .data_bits        = UART_DATA_8_BITS,
            .stop_bits        = (fmt == OUT_FMT_8N2) ? UART_STOP_BITS_2
                                                     : UART_STOP_BITS_1,
            .parity           = par,
            .response_tout_ms = RESPONSE_TOUT_MS,
        },
    };
    esp_err_t err = mbc_slave_create_serial(&comm, &in->ctx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "slave create (serial): %s", esp_err_to_name(err));
        return err;
    }

    if ((err = areas_and_start(in->ctx)) != ESP_OK) return err;

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

    xSemaphoreTake(in->lock, portMAX_DELAY);
    in->stat.running = true;
    in->stat.address = addr;
    in->stat.baud    = baud;
    in->stat.format  = out_common_format_name(fmt);
    in->stat.port    = 0;
    xSemaphoreGive(in->lock);

    if (xTaskCreate(mb_task, "out_mb_rtu", 3072, in, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Modbus RTU slave up: address %d, %d baud %s, "
                  "TX %d RX %d DE %d (hardware RTS)",
             addr, baud, out_common_format_name(fmt),
             BOARD_PIN_RS485_TX, BOARD_PIN_RS485_RX, BOARD_PIN_RS485_DE);
    return ESP_OK;
}
#endif /* BOARD_HAS_RS485 */

esp_err_t out_modbus_start(void)
{
    /* TWO INDEPENDENT OUTPUTS. RTU is the trunk's protocol, at most one of
     * which runs; TCP is a separate switch. Both at once is the case this
     * whole esp-modbus 2.x migration was for.
     *
     * The address is shared: it is the RTU station address and the TCP unit
     * id, one number because a site thinks of it as one. */
    const bool want_rtu = (out_common_rs485() == OUT_PROTO_MODBUS_RTU);
    const bool want_tcp = out_common_tcp_enabled();
    if (!want_rtu && !want_tcp) {
        return ESP_OK;                      /* not ours: cost the board nothing */
    }

    const int addr = out_common_address();
    esp_err_t worst = ESP_OK;

    if (want_rtu) {
#if BOARD_HAS_RS485
        esp_err_t e = start_rtu(addr);
#else
        ESP_LOGE(TAG, "Modbus RTU selected, but this board has no RS485 transceiver");
        esp_err_t e = ESP_ERR_NOT_SUPPORTED;
#endif
        /* Deliberately NOT a return. These outputs are independent, so a
         * trunk that will not come up must not take Modbus TCP down with it
         * -- the site watching over the network keeps working, and the
         * failure is visible on the page and in the log. */
        if (e != ESP_OK) worst = e;
    }
    if (want_tcp) {
        esp_err_t e = start_tcp(addr);
        if (e != ESP_OK && worst == ESP_OK) worst = e;
    }

    ESP_LOGI(TAG, "bus commands %s (coil %d starts a quick self-test)",
             out_common_cmd_enabled() ? "ENABLED" : "off", CMD_COIL_ADDR);
    return worst;
}

static void get_one(inst_t *in, out_modbus_stat_t *out)
{
    if (!in->lock) { memset(out, 0, sizeof(*out)); return; }
    xSemaphoreTake(in->lock, portMAX_DELAY);
    *out = in->stat;
    xSemaphoreGive(in->lock);
}

void out_modbus_get_rtu(out_modbus_stat_t *out) { get_one(&s_rtu, out); }
void out_modbus_get_tcp(out_modbus_stat_t *out) { get_one(&s_tcp, out); }
