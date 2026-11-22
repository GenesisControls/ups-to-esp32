/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Modbus slave on the RS485 trunk, serving the point set from out_common.
 *
 * READ-ONLY ON THE BUS. Every write is refused, including FC5/6/15/16/22/23,
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
    uint32_t requests;        /* frames addressed to us and answered */
    uint32_t writes_refused;  /* attempts to write; always refused */
    int64_t  last_request_us; /* 0 if the trunk has never spoken to us */
} out_modbus_stat_t;

void out_modbus_get(out_modbus_stat_t *out);

#endif /* GENESIS_OUT_MODBUS_H */
