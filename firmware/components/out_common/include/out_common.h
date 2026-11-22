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

/* THESE TWO ARE ADDRESSES, NOT CHOICES, so they start at 0 and stay there.
 * Modbus addressing is 0-based and the other track's map fixes every one of
 * them; renumbering would move every point at every commissioned site. The
 * from-1 rule above applies to enums we are free to number. */
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
/* Numbered from 1, like every choice enum here: a zero-valued member cannot
 * be told apart from an NVS key that was never written or a JSON field that
 * was never sent, and both of those happen. */
typedef enum {
    OUT_PROTO_NONE = 1,         /* nothing on the bus; the page still works */
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

/* Serial framing on the trunk. One setting rather than separate parity and
 * stop-bit fields: a BMS tool asks for "8N1" and the pair only has four
 * combinations anyone uses, so splitting them would mostly offer invalid ones.
 *
 * 8N1 is the default and what nearly every BMS master is set to. Note that
 * the Modbus RTU spec actually calls for TWO stop bits when there is no
 * parity, to keep eleven bits per character -- 8N2 is here for a master that
 * holds to that, because a device framing 8N1 against a strict 8N2 master
 * produces frame errors that look exactly like a wrong baud rate. */
typedef enum {
    OUT_FMT_8N1 = 1,            /* default */
    OUT_FMT_8E1,
    OUT_FMT_8O1,
    OUT_FMT_8N2,                /* what the Modbus RTU spec asks for */
} out_fmt_t;

out_fmt_t   out_common_format(void);
esp_err_t   out_common_set_format(out_fmt_t f);
const char *out_common_format_name(out_fmt_t f);

/* What register 6 does when the UPS link is lost. */
typedef enum {
    /* 65535, matching the Python bridge's default and the handover document.
     * Niagara sees the heartbeat stop dead with everything else. */
    OUT_HB_INVALID = 1,
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

/* Names and units for the page's register overview.
 *
 * Here, beside the enums, rather than in the web UI: the table a
 * commissioning engineer reads off the screen and ticks against the Niagara
 * point sheet must not be able to drift from the map this file implements.
 * A reserved address has no name -- it does not exist. */
const char *out_common_discrete_name(int i);
const char *out_common_reg_name(int i);
const char *out_common_reg_unit(int i);     /* "" where unitless */

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

/* COMMANDS FROM THE BUS. Off by default, and deliberately so.
 *
 * The brief's original rule was "read-only, no UPS commands over
 * Modbus/BACnet", reversed by Tobias on 2026-11-22 to allow a battery
 * self-test trigger. The rule existed for a reason that has not gone away:
 * an RS485 trunk is unauthenticated, a self-test puts the site's load on
 * battery, and a misconfigured master repeating the write would cycle the
 * pack. So this is per-board opt-in rather than a build-wide default, and a
 * board nobody enabled stays read-only exactly as before.
 *
 * The command itself carries its own guards in out_modbus: quick test only,
 * refused unless the UPS's own descriptor reports UPS_CAP_TEST, rate
 * limited, and logged every time. */
bool      out_common_cmd_enabled(void);
esp_err_t out_common_set_cmd_enabled(bool on);

#endif /* GENESIS_OUT_COMMON_H */
