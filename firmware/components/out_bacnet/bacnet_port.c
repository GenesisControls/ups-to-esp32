/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * The platform layer bacnet-stack needs underneath MS/TP: an RS485 byte pipe
 * and a millisecond clock.
 *
 * Written against the documented interfaces (struct dlmstp_rs485_driver in
 * datalink/dlmstp.h, mstimer_now() in basic/sys/mstimer.h) rather than adapted
 * from bacnet-stack's own ports/esp32 -- see README-GENESIS.md in the bacnet
 * component for why. The upstream port was read as reference; its approach of
 * letting the UART peripheral drive DE in hardware half-duplex mode is the
 * same one out_modbus uses, and is kept here for the same reason.
 */

#include "bacnet_port.h"

#include "board.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "bacnet/basic/sys/mstimer.h"

#if BOARD_HAS_RS485

static const char *TAG = "bac485";

/* MS/TP's own default. Overridden from the web UI before the datalink starts,
 * so this value only ever applies to a board nobody has configured. */
#define BAUD_DEFAULT 38400u

/* One frame is at most DLMSTP_MPDU_MAX; two of them of slack means a master
 * that bursts cannot overrun us between reads. */
#define RX_BUFFER 1024

static uint32_t          s_baud = BAUD_DEFAULT;
static bool              s_transmitting;
static volatile uint32_t s_bytes_tx;
static volatile uint32_t s_bytes_rx;
static struct mstimer    s_silence;
static bool              s_up;

/* ---- the millisecond clock the stack is built on ---------------------- */

/* mstimer.c declares these and expects the platform to define them. The whole
 * MS/TP state machine is timed off this, so it has to be monotonic and cheap:
 * esp_timer_get_time() is both, and unlike a FreeRTOS tick count it does not
 * depend on configTICK_RATE_HZ being 1000. */
unsigned long mstimer_now(void)
{
    return (unsigned long)(esp_timer_get_time() / 1000);
}

void mstimer_init(void)
{
    /* esp_timer is running before app_main, so there is nothing to start. */
}

/* ---- the RS485 byte pipe --------------------------------------------- */

/* NOTHING IN BACNET-STACK CALLS THIS.
 *
 * It fills in struct dlmstp_rs485_driver::init, which reads like the stack's
 * way of bringing the hardware up -- and the stack never calls it. Not from
 * dlmstp_init(), not from anywhere in src/. out_bacnet_start() calls it
 * directly, and must: see the comment there for what a board does when this
 * never runs, which is not "BACnet stays quiet". */
void bacnet_rs485_init(void)
{
    const uart_config_t cfg = {
        .baud_rate  = (int)s_baud,
        .data_bits  = UART_DATA_8_BITS,
        /* MS/TP is 8N1 on the wire, always. Unlike Modbus there is no parity
         * choice to expose: ANSI/ASHRAE 135 fixes the character format, so the
         * data-format setting in the web UI does not apply here. */
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(BOARD_RS485_UART, &cfg);
    if (err != ESP_OK) { ESP_LOGE(TAG, "uart_param_config: %s", esp_err_to_name(err)); return; }

    /* DE on the peripheral's RTS, as out_modbus does. Keying the transceiver
     * from software is a race that clips the start of a frame at the wire
     * while everything on the board still looks correct. */
    err = uart_set_pin(BOARD_RS485_UART, BOARD_PIN_RS485_TX, BOARD_PIN_RS485_RX,
                       BOARD_PIN_RS485_DE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) { ESP_LOGE(TAG, "uart_set_pin: %s", esp_err_to_name(err)); return; }

    /* No event queue: the MS/TP state machine pulls one byte at a time from
     * its own task and an event queue would only add a second path to the
     * same bytes. */
    err = uart_driver_install(BOARD_RS485_UART, RX_BUFFER, 0, 0, NULL, 0);
    if (err != ESP_OK) { ESP_LOGE(TAG, "uart_driver_install: %s", esp_err_to_name(err)); return; }

    err = uart_set_mode(BOARD_RS485_UART, UART_MODE_RS485_HALF_DUPLEX);
    if (err != ESP_OK) { ESP_LOGE(TAG, "uart_set_mode: %s", esp_err_to_name(err)); return; }

    s_transmitting = false;
    s_bytes_tx = 0;
    s_bytes_rx = 0;
    s_up = true;
    bacnet_rs485_silence_reset();

    ESP_LOGI(TAG, "RS485 up for MS/TP: %lu baud 8N1, TX %d RX %d DE %d (hardware RTS)",
             (unsigned long)s_baud, BOARD_PIN_RS485_TX, BOARD_PIN_RS485_RX,
             BOARD_PIN_RS485_DE);
}

void bacnet_rs485_send(const uint8_t *payload, uint16_t len)
{
    if (!s_up || !payload || len == 0) {
        /* Reset the silence timer even on the path that sends nothing.
         *
         * MS/TP's every timeout is measured as "how long has the line been
         * quiet", and dlmstp_receive() runs the master state machine in a
         * `while (MSTP_Master_Node_FSM(port))` loop that only ends when the
         * machine has a timer left to wait on. A send that neither puts bytes
         * on the wire nor moves the silence timer leaves every timeout
         * permanently expired, so every transition is a "fast" one and that
         * loop never returns -- which is a spin inside datalink_receive(),
         * not a quiet bus.
         *
         * Our caller cannot yield while that happens, so the idle task
         * starves and the watchdog reboots the board. Two lines of agreement
         * with the state machine are cheaper than depending on the UART never
         * failing. */
        bacnet_rs485_silence_reset();
        return;
    }

    s_transmitting = true;
    uart_write_bytes(BOARD_RS485_UART, (const char *)payload, len);
    /* MS/TP timing is measured from the END of our frame, so the silence timer
     * must not start until the last bit is actually on the wire. At 9600 baud
     * a maximum frame is ~550 ms, so the wait is generous rather than tight;
     * it is a backstop against a wedged peripheral, not the normal path. */
    uart_wait_tx_done(BOARD_RS485_UART, pdMS_TO_TICKS(1000));
    s_bytes_tx += len;
    s_transmitting = false;
    bacnet_rs485_silence_reset();
}

bool bacnet_rs485_read(uint8_t *buf)
{
    if (!s_up) return false;

    size_t buffered = 0;
    if (uart_get_buffered_data_len(BOARD_RS485_UART, &buffered) != ESP_OK) return false;
    if (buffered == 0) return false;
    if (!buf) return true;                  /* a probe, not a read */

    if (uart_read_bytes(BOARD_RS485_UART, buf, 1, 0) != 1) return false;
    s_bytes_rx++;
    bacnet_rs485_silence_reset();
    return true;
}

bool bacnet_rs485_transmitting(void)
{
    return s_transmitting;
}

uint32_t bacnet_rs485_baud(void)
{
    return s_baud;
}

bool bacnet_rs485_baud_set(uint32_t baud)
{
    if (baud == 0) return false;
    s_baud = baud;
    /* Before init this only records the value; uart_param_config applies it. */
    if (!s_up) return true;
    return uart_set_baudrate(BOARD_RS485_UART, (int)baud) == ESP_OK;
}

uint32_t bacnet_rs485_silence_ms(void)
{
    return (uint32_t)mstimer_elapsed(&s_silence);
}

void bacnet_rs485_silence_reset(void)
{
    mstimer_set(&s_silence, 0);
}

/* Whether the UART is actually up. The stack has no way to ask and no way to
 * report it, so out_bacnet_start() checks this itself and refuses to start
 * the datalink without it. */
bool bacnet_rs485_up(void) { return s_up; }

uint32_t bacnet_rs485_bytes_tx(void) { return s_bytes_tx; }
uint32_t bacnet_rs485_bytes_rx(void) { return s_bytes_rx; }

#endif /* BOARD_HAS_RS485 */
