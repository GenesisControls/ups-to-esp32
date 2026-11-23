/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * The RS485 byte pipe under BACnet MS/TP. These functions are what fills in
 * bacnet-stack's struct dlmstp_rs485_driver; nothing else should call them.
 *
 * Note the division of labour with out_modbus: both drive the same UART and
 * the same transceiver, and exactly one protocol is active per boot, so they
 * never contend. Neither one tears the other down.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Opens the UART and puts it in hardware half-duplex mode. Call once, before
 * dlmstp_init(); the driver struct points at it as .init so the datalink does
 * that itself. */
void     bacnet_rs485_init(void);
/* True once the UART is configured and installed. bacnet_rs485_init() reports
 * failure only in the log, because the stack's driver struct wants a void
 * init; this is how a caller finds out. */
bool     bacnet_rs485_up(void);

/* Sends a whole frame and blocks until the last bit has left the shift
 * register, because MS/TP's inter-frame timing starts there and not when the
 * bytes reach the FIFO. */
void     bacnet_rs485_send(const uint8_t *payload, uint16_t len);

/* One byte, if there is one. buf NULL probes without consuming. */
bool     bacnet_rs485_read(uint8_t *buf);

bool     bacnet_rs485_transmitting(void);
uint32_t bacnet_rs485_baud(void);

/* Settable before init (records only) or after (applies immediately): the
 * datalink changes baud through this when autobaud eventually exists. */
bool     bacnet_rs485_baud_set(uint32_t baud);

/* Milliseconds since the last byte in either direction. The MS/TP state
 * machine runs its token timing off this. */
uint32_t bacnet_rs485_silence_ms(void);
void     bacnet_rs485_silence_reset(void);

/* Wire-level counters, for the commissioning view. */
uint32_t bacnet_rs485_bytes_tx(void);
uint32_t bacnet_rs485_bytes_rx(void);

#ifdef __cplusplus
}
#endif
