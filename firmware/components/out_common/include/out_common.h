/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * The Genesis point set: one place that turns ups_data_t into the numbers
 * Niagara reads, so Modbus and BACnet cannot drift apart.
 *
 * WHY THIS EXISTS. Two outputs (out_modbus, out_bacnet) publish the same
 * facts, and a second track -- the Windows NUT + pymodbus bridge in
 * ups-modbus-monitoring -- publishes them too, to the same Niagara point
 * sheet. Three renderings of one contract is three chances to disagree about
 * what 65535 means or which bit is "on battery". So the contract lives here,
 * once, with no I/O in it, and the outputs only format what this produces.
 *
 * THE OTHER TRACK OWNS THE NUMBERING. ups-modbus-monitoring/README.md, under
 * "The map", is authoritative: addresses, units, the status bitfield's width
 * and the comm-loss behaviour. Registers 0-7 are already wired into Niagara
 * at live sites. Do not change them here; change them there, with that track,
 * or both halves of the fleet disagree.
 *
 * Address space, signed off 2026-11-20:
 *   discretes 0-10    the UPS's state
 *   registers 0-7     the original points, LIVE at sites today
 *   registers 8-15    facts about the UPS      (12-15 reserved)
 *   registers 16-23   facts about the adapter  (19-23 reserved)
 */
#ifndef GENESIS_OUT_COMMON_H
#define GENESIS_OUT_COMMON_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define OUT_DISCRETE_N  11      /* 0..10  */
#define OUT_REG_N       24      /* 0..23  */

/* "Not reported by this UPS", or "comm lost". Real measurements are clamped
 * to OUT_REG_MAX_REAL so this value can never be mistaken for one. */
#define OUT_ABSENT      65535u
#define OUT_REG_MAX_REAL 65534u

enum {                          /* discrete inputs (FC2), mirrored as coils  */
    OUT_D_ONLINE = 0,
    OUT_D_ON_BATTERY,           /* the one every site alarms on              */
    OUT_D_LOW_BATTERY,
    OUT_D_REPLACE_BATTERY,
    OUT_D_OVERLOAD,
    OUT_D_COMM_OK,
    OUT_D_AVR_BOOST,
    OUT_D_AVR_TRIM,
    OUT_D_CHARGING,
    OUT_D_BATTERY_AGE_ALARM,
    OUT_D_OUTPUT_NOT_PROTECTING, /* OFF or BYPASS: the load is NOT on the UPS */
};

enum {                          /* input registers (FC4), mirrored as holding */
    OUT_R_CHARGE_PCT = 0,
    OUT_R_RUNTIME_S,
    OUT_R_LOAD_PCT,
    OUT_R_INPUT_V10,
    OUT_R_OUTPUT_V10,
    OUT_R_BATTERY_V10,
    OUT_R_HEARTBEAT,
    OUT_R_STATUS_BITS,          /* bits 0-5 ONLY -- see the note below        */
    OUT_R_BATTERY_AGE_D,
    OUT_R_TEST_RESULT,
    OUT_R_TRANSFER_LOW_V10,
    OUT_R_TRANSFER_HIGH_V10,
    /* 12-15 reserved for further UPS facts */
    OUT_R_FW_VERSION = 16,
    OUT_R_RESET_COUNT,
    OUT_R_RESET_REASON,
    /* 19-23 reserved for further adapter facts */
};

/* WHICH PROTOCOL IS ON THE BUS. Exactly one, ever.
 *
 * Kept here rather than in either output so both read the same answer and
 * neither can start while the other holds the trunk. Each output is started
 * unconditionally from main.c, asks this, and creates no task and allocates
 * nothing unless it is the one selected -- the pattern upstream's snmp.c
 * already uses, so a board that never speaks BACnet pays nothing for it.
 *
 * The IP protocols are offered only while an interface actually has an
 * address. NOT keyed to the board type: the brief originally said Modbus TCP
 * and BACnet/IP were "Ethernet board only", but an RS485 board that joins a
 * Wi-Fi can serve them perfectly well, and one site will. Ask the network,
 * not the part number. */
typedef enum {
    OUT_PROTO_NONE = 0,         /* nothing on the bus; the page still works */
    OUT_PROTO_MODBUS_RTU,       /* RS485 */
    OUT_PROTO_BACNET_MSTP,      /* RS485 */
    OUT_PROTO_MODBUS_TCP,       /* needs an IP address */
    OUT_PROTO_BACNET_IP,        /* needs an IP address */
} out_proto_t;

out_proto_t out_common_proto(void);
esp_err_t   out_common_set_proto(out_proto_t p);
const char *out_common_proto_name(out_proto_t p);

/* The station address, shared by Modbus RTU's slave/unit ID and BACnet
 * MS/TP's MAC because a site thinks of it as one number: "which device is
 * this on the trunk". Default 4.
 *
 * Valid 1-247 for Modbus, 0-127 for MS/TP masters, so 1-127 is safe for
 * both and is what this accepts. Two adapters on one segment at the same
 * address collide silently, so deriving a default from the MAC -- as the AP
 * SSID already does -- is worth revisiting if these ever share a trunk. */
#define OUT_ADDR_DEFAULT 4
int       out_common_address(void);
esp_err_t out_common_set_address(int addr);

/* 0 means autobaud: listen, lock on, and re-scan after silence. Anything else
 * is a fixed rate. Default 0. Autobaud is its own piece of work and does not
 * exist yet, so an output seeing 0 today should fall back to 9600 and say so
 * rather than stay silent. */
#define OUT_BAUD_AUTO 0
int       out_common_baud(void);
esp_err_t out_common_set_baud(int baud);

/* What register 6 does when the UPS link is lost. */
typedef enum {
    /* 65535, matching the Python bridge's default and the handover document.
     * Niagara sees the heartbeat stop dead with everything else. */
    OUT_HB_INVALID = 0,
    /* Keep counting. Lets a site tell "adapter alive, UPS link dead" from
     * "adapter dead", which the first cannot otherwise be distinguished from.
     * Still open with the other track; off until they agree. */
    OUT_HB_RUN,
} out_hb_policy_t;

typedef struct {
    bool     comm_ok;                       /* same as discrete 5            */
    bool     discrete[OUT_DISCRETE_N];
    uint16_t reg[OUT_REG_N];

    /* Which addresses this firmware actually implements.
     *
     * A reserved or unimplemented address MUST NOT appear in the datastore.
     * The other track is explicit: reading one returns Modbus exception 02,
     * which is a different answer from 65535 and a site has to tell them
     * apart -- 65535 says "this UPS does not report it", exception 02 says
     * "this adapter does not have it". An output that registered 0-23 and
     * filled the gaps with 65535 would quietly erase that distinction. */
    bool     reg_live[OUT_REG_N];
} out_points_t;

/* Load settings, seed the boot counter, start the 1 Hz refresh. After
 * ups_hid_start(), like every other consumer. */
esp_err_t out_common_init(void);

/* Copy the current point set. Thread-safe; both outputs read the same
 * snapshot, so RTU and TCP cannot answer differently in the same second. */
void out_common_get(out_points_t *out);

/* The battery's last change date, "YYYY-MM-DD", or "" to fall back to what
 * the UPS reports in battery.mfr.date. Consumer UPSes cannot store this, so
 * the adapter keeps it. Persisted in NVS. */
esp_err_t out_common_set_battery_date(const char *yyyy_mm_dd);
void      out_common_get_battery_date(char *out, size_t cap);

/* Years before discrete 9 (battery age alarm) asserts. 0 disables it. */
esp_err_t out_common_set_battery_life_years(int years);

esp_err_t out_common_set_hb_policy(out_hb_policy_t p);
out_hb_policy_t out_common_hb_policy(void);

#endif /* GENESIS_OUT_COMMON_H */
