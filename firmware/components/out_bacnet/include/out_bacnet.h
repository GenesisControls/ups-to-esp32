/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * BACnet MS/TP output. Serves the out_common point set as BACnet objects on
 * the RS485 trunk, read-only but for BV1.
 *
 * Instance numbering is the Modbus address + 1 -- BI1..BI11 for discretes
 * 0..10, AI<n+1> for register n -- so one map covers both protocols. BV1 is
 * the exception and the only writable object: writing it Active starts a
 * battery self-test, the BACnet counterpart of Modbus coil 100. See the
 * comment at the top of out_bacnet.c for why the point numbering differs from
 * the brief's proposed BACnet table, for how Reliability replaces Modbus's
 * 65535, and for why BV1 is a Value rather than an Output.
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool     running;
    int      mac;                /* MS/TP station address, 0-127 */
    int      baud;
    uint32_t instance;           /* BACnet device object instance, as in use */
    int      info_frames;        /* Nmax_info_frames actually applied */
    int      max_master;         /* Nmax_master actually applied */
    char     device_name[32];    /* what a BMS discovers us as */
    uint32_t requests;           /* NPDUs handled since boot */
    int64_t  last_request_us;    /* esp_timer stamp, 0 if never */
    uint32_t bytes_tx;
    uint32_t bytes_rx;
    uint32_t tests_started;      /* self-tests started by a write to BV1 */
} out_bacnet_stat_t;

/* Starts the datalink and the MS/TP task, but only if BACnet MS/TP is the
 * selected protocol; otherwise returns ESP_OK having done nothing, so main
 * can call every output unconditionally.
 *
 * Call after out_common_init(). Needs the RS485 UART, which means it and
 * out_modbus must never both start -- one protocol is active per boot. */
esp_err_t out_bacnet_start(void);

/* A snapshot for the web UI's commissioning view. Safe before start(): it
 * zeroes the struct. */
void      out_bacnet_get(out_bacnet_stat_t *out);

#ifdef __cplusplus
}
#endif
