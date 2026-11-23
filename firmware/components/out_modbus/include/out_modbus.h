/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Modbus slave on the RS485 trunk, serving the point set from out_common.
 *
 * READ-ONLY ON THE BUS, with ONE exception: coil 100 starts a quick battery
 * self-test, and only on a board whose owner has switched commands on. Every
 * other write is refused, including FC5/6/15/16/22/23,
 * and refused by the datastore rather than by convention: a BMS integrator
 * who points a write at this device gets an exception, not a silently
 * ignored command and a value that never changes.
 *
 * One protocol at a time. This is started unconditionally from main.c, asks
 * out_common_proto(), and creates nothing at all unless Modbus is the one
 * selected -- upstream's snmp.c works the same way, so a board serving
 * BACnet pays nothing for this.
 *
 * ADDRESSES BELONG TO THE OTHER TRACK. ups-modbus-monitoring/README.md, under
 * "The map", is authoritative, and registers 0-7 are already wired into
 * Niagara at live sites. This component only renders out_common's point set
 * onto the wire; it decides nothing about what lives where.
 */
#ifndef GENESIS_OUT_MODBUS_H
#define GENESIS_OUT_MODBUS_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

/* Start the slave if out_common_proto() selects a Modbus flavour, else do
 * nothing and return ESP_OK. Call after out_common_init(). */
esp_err_t out_modbus_start(void);

/* For the status page: what the bus is actually doing, so a commissioning
 * engineer can tell "wrong baud" from "wrong address" from "no cable"
 * without a protocol analyser. */
typedef struct {
    bool     running;
    int      address;
    int      baud;            /* the rate in use, resolved from auto */
    const char *format;       /* "8N1" and friends, as actually configured */
    int      port;            /* TCP only: the listening port. 0 on RTU. */
    uint32_t requests;        /* frames addressed to us and answered */
    uint32_t writes_refused;  /* writes to read-only addresses; undone */
    uint32_t tests_started;   /* self-tests triggered from the bus */
    int64_t  last_request_us; /* 0 if the trunk has never spoken to us */

    /* Per access type, because WHICH half of a point sheet a master is
     * reading is the fastest way to find a half-configured one: input
     * registers but never discretes means FC2 was never mapped. */
    uint32_t rd_input, rd_holding, rd_discrete, rd_coil;

    /* Requests in the last 60 s: shows the master actually polling, and at
     * what rate, rather than just that it once did. */
    uint32_t per_min;

    /* RS485 half-duplex collisions, from uart_get_collision_flag(). Two
     * devices keying the line at once -- nearly always a duplicate address
     * on the trunk. The one genuine wire-level fault this stack exposes. */
    uint32_t collisions;

    /* NOT MEASURABLE, and deliberately absent rather than always zero:
     * CRC and framing error counts. esp-modbus 1.0 drops a malformed or
     * mis-addressed frame inside the stack, and mbc_slave_check_event() only
     * fires on a SUCCESSFUL access -- there is no hook for a frame that
     * failed CRC. A counter reading zero for ever would read as "healthy"
     * when it means "not counted", which is worse than not offering it.
     * Wrong baud or format shows up instead as silence while the trunk's
     * other devices work; a duplicate address shows up as collisions. */
} out_modbus_stat_t;

void out_modbus_get(out_modbus_stat_t *out);

#endif /* GENESIS_OUT_MODBUS_H */
