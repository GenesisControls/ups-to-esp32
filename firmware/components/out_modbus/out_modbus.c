/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "out_modbus.h"
#include "out_common.h"
#include "board.h"
#include "mbcontroller.h"
#include "driver/uart.h"
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
#define REFRESH_MS      1000

/* The register image the stack serves from. esp-modbus answers reads straight
 * out of these, so they are refreshed from out_common rather than built per
 * request -- a Modbus master polling hard must never be able to make us poll
 * the UPS hard. */
static uint16_t s_ireg[OUT_REG_N];
static uint16_t s_hreg[OUT_REG_N];          /* the FC3 mirror of the same data */
static uint8_t  s_disc[(OUT_DISCRETE_N + 7) / 8];
static uint8_t  s_coil[(OUT_DISCRETE_N + 7) / 8];

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
            s_stat.last_request_us = esp_timer_get_time();
            if (ev & (MB_EVENT_HOLDING_REG_WR | MB_EVENT_COILS_WR)) {
                s_stat.writes_refused++;
            }
            xSemaphoreGive(s_stat_lock);
            if (ev & (MB_EVENT_HOLDING_REG_WR | MB_EVENT_COILS_WR)) {
                ESP_LOGW(TAG, "a write arrived on a read-only device; undoing it");
                refresh();                  /* put the value straight back */
            }
        }

        if (xTaskGetTickCount() - next >= period) {
            next = xTaskGetTickCount();
            refresh();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

esp_err_t out_modbus_start(void)
{
    out_proto_t proto = out_common_proto();
    if (proto != OUT_PROTO_MODBUS_RTU && proto != OUT_PROTO_MODBUS_TCP) {
        return ESP_OK;                      /* not ours: cost the board nothing */
    }
    if (proto == OUT_PROTO_MODBUS_TCP) {
        /* Needs an interface with an address, and the settings page only
         * offers it when there is one. Not wired yet. */
        ESP_LOGW(TAG, "Modbus TCP is selected but not implemented yet");
        return ESP_OK;
    }
#if !BOARD_HAS_RS485
    ESP_LOGE(TAG, "Modbus RTU selected, but this board has no RS485 transceiver");
    return ESP_ERR_NOT_SUPPORTED;
#else
    s_stat_lock = xSemaphoreCreateMutex();
    if (!s_stat_lock) return ESP_ERR_NO_MEM;

    int baud = out_common_baud();
    if (baud == OUT_BAUD_AUTO) {
        baud = BAUD_FALLBACK;
        ESP_LOGW(TAG, "baud is set to auto but autobaud is not built yet: using %d", baud);
    }
    const int addr = out_common_address();

    esp_err_t err = mbc_slave_init(MB_PORT_SERIAL_SLAVE, &s_handler);
    if (err != ESP_OK) { ESP_LOGE(TAG, "slave init: %s", esp_err_to_name(err)); return err; }

    mb_communication_info_t comm = {
        .mode       = MB_MODE_RTU,
        .slave_addr = (uint8_t)addr,
        .port       = BOARD_RS485_UART,
        .baudrate   = (uint32_t)baud,
        .parity     = UART_PARITY_DISABLE,
    };
    if ((err = mbc_slave_setup(&comm)) != ESP_OK) {
        ESP_LOGE(TAG, "slave setup: %s", esp_err_to_name(err));
        return err;
    }

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
    }

    if ((err = mbc_slave_start()) != ESP_OK) {
        ESP_LOGE(TAG, "slave start: %s", esp_err_to_name(err));
        return err;
    }

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

    xSemaphoreTake(s_stat_lock, portMAX_DELAY);
    s_stat.running = true;
    s_stat.address = addr;
    s_stat.baud    = baud;
    xSemaphoreGive(s_stat_lock);

    if (xTaskCreate(mb_task, "out_mb", 3072, NULL, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Modbus RTU slave up: address %d, %d baud 8N1, "
                  "TX %d RX %d DE %d (hardware RTS)",
             addr, baud, BOARD_PIN_RS485_TX, BOARD_PIN_RS485_RX, BOARD_PIN_RS485_DE);
    return ESP_OK;

fail:
    ESP_LOGE(TAG, "register area: %s", esp_err_to_name(err));
    return err;
#endif /* BOARD_HAS_RS485 */
}

void out_modbus_get(out_modbus_stat_t *out)
{
    if (!s_stat_lock) { memset(out, 0, sizeof(*out)); return; }
    xSemaphoreTake(s_stat_lock, portMAX_DELAY);
    *out = s_stat;
    xSemaphoreGive(s_stat_lock);
}
