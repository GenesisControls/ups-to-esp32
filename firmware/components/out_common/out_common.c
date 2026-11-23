/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "out_common.h"
#include "ups_hid.h"
#include "ups_nutstatus.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static const char *TAG = "outcom";

/* The namespace every other component shares, so netmgr_factory_reset()
 * erases these with the rest. Keys are prefixed, as the house does. */
#define NVS_NS          "upsa"
#define KEY_BAT_DATE    "oc_batdate"
#define KEY_BAT_LIFE    "oc_batlife"
#define KEY_HB          "oc_hb"
#define KEY_BOOTS       "oc_boots"
#define KEY_PROTO       "oc_proto"
#define KEY_ADDR        "oc_addr"
#define KEY_BAUD        "oc_baud"
#define KEY_FMT         "oc_fmt"
#define KEY_CMD         "oc_cmd"
#define KEY_DEEP        "oc_deep"
#define KEY_DEVID       "oc_devid"
#define KEY_INFOFR      "oc_infofr"
#define KEY_MAXMAST     "oc_maxmast"
#define KEY_MBPORT      "oc_mbport"
#define KEY_TCPUID      "oc_tcpuid"
#define KEY_TCPEN       "oc_tcpen"

/* Comm-OK's threshold. The other track's README specifies 10 s, and upstream
 * independently arrives at the same number: FAILURES_BEFORE_RESET (5) polls
 * at POLL_INTERVAL_MS (2000) is "~10s of silence" in its own comment, after
 * which ups_hid drops `attached` and blanks the readings. The time test here
 * is belt and braces for the one path where ups_hid holds poll_failures at
 * zero while not polling (usb_guard, enumeration stuck): there `attached`
 * can stay true with readings that stopped advancing. */
#define STALE_US        (10LL * 1000 * 1000)

#define REFRESH_US      (1000LL * 1000)     /* heartbeat cadence: +1 per second */

static SemaphoreHandle_t s_lock;
static out_points_t      s_pts;
static esp_timer_handle_t s_timer;

static uint16_t          s_heartbeat;
static uint16_t          s_boots;
static int               s_bat_life_years = 5;
static char              s_bat_date[11];    /* "YYYY-MM-DD", "" for unset */
static out_hb_policy_t   s_hb = OUT_HB_INVALID;
static out_proto_t       s_proto = OUT_PROTO_NONE;   /* one of RTU/MSTP/BIP */
static bool              s_tcp_en;                  /* Modbus TCP, alongside */
static int               s_addr = OUT_ADDR_DEFAULT;
static int               s_baud = OUT_BAUD_AUTO;
static out_fmt_t         s_fmt = OUT_FMT_8N1;
static bool              s_cmd_enabled = true;   /* self-test from the bus */
/* Off by default, unlike the quick test: see out_common.h for why a runtime
 * calibration is not the same kind of request. */
static bool              s_deep_enabled;
static uint32_t          s_devid = OUT_DEVID_DERIVE;
static int               s_info_frames = OUT_INFOFRAMES_DEFAULT;
static int               s_max_master = OUT_MAXMASTER_DEFAULT;
static int               s_mb_tcp_port = OUT_MB_TCP_PORT_DEFAULT;
static int               s_tcp_uid = OUT_TCP_UID_ANY;

/* ---- helpers ---------------------------------------------------------- */

/* A measurement, scaled and clamped. Clamping to 65534 is what keeps 65535
 * meaning exactly one thing; a real 65535 V x10 would be indistinguishable
 * from "not reported", and the other track clamps for the same reason. */
static uint16_t meas(bool live, float v, float scale)
{
    if (!live || !ups_valid(v)) return OUT_ABSENT;
    long x = lroundf(v * scale);
    if (x < 0) x = 0;
    if (x > (long)OUT_REG_MAX_REAL) x = (long)OUT_REG_MAX_REAL;
    return (uint16_t)x;
}

/* Register 9.
 *
 * THE TWO TRACKS DO NOT USE THE SAME ENUM, so this is a table and not a cast.
 * upstream's ups_test_result_t (ups_data.h) reads 4=ABORTED, 5=IN_PROGRESS,
 * 6=NOT_RUN; the other track's published encoding is 4=in progress,
 * 5=aborted, 6=scheduled. Casting would report an aborted test as running and
 * a running one as aborted.
 *
 * Upstream's table is the better-evidenced of the two -- ups_data.h records
 * that it was taken from NUT's usbhid-ups and verified against the shipped
 * driver binary, after inferring it from the HID spec made a healthy battery
 * read FAILED. The other track calls its own table provisional. Until they
 * settle it, this maps upstream's reading onto the published numbering,
 * because the published numbering is what Niagara was pointed at.
 *
 * UNKNOWN IS 7, NOT 0 -- changed 2026-11-23 on Tobias's instruction.
 *
 * Every enumerated point in this project starts at 1, so that the number a
 * BMS reads over BACnet is the number a BMS reads over Modbus. A BACnet
 * Multi-State Input's present-value is 1..N by ANSI/ASHRAE 135, so an
 * encoding with a meaningful 0 would have forced an off-by-one between the
 * two protocols for this one register -- exactly the kind of difference that
 * gets a commissioning engineer to map a 3 onto a 3 and be wrong.
 *
 * So "unknown" moved off 0 and onto 7. 1-6 are untouched, which matters: the
 * other track's published table owns them, and 6 stays "scheduled" there
 * rather than being reused. 0 is now simply not produced.
 *
 * NOT_RUN still has no counterpart over there ("scheduled" is a different
 * claim), so it lands on unknown -- which is what NOT_RUN means, and is what
 * the bench APC returns at rest. We never emit 6; no upstream state maps to
 * it. */
#define TEST_UNKNOWN  7u

static uint16_t test_result(bool live, uint8_t upstream_value)
{
    if (!live) return OUT_ABSENT;
    switch (upstream_value) {
    case UPS_TEST_NO_TEST:     return TEST_UNKNOWN;
    case UPS_TEST_PASSED:      return 1;
    case UPS_TEST_WARNING:     return 2;
    case UPS_TEST_ERROR:       return 3;
    case UPS_TEST_IN_PROGRESS: return 4;
    case UPS_TEST_ABORTED:     return 5;
    case UPS_TEST_NOT_RUN:     return TEST_UNKNOWN;
    default:                   return OUT_ABSENT;
    }
}

/* Register 18, 1-based for the same reason.
 *
 * Its encoding is ours: the other track's README says register 18 is
 * "track-specific in its encoding, not its meaning" and publishes 65535
 * there. esp_reset_reason_t starts at 0 and is append-only, so one added at
 * publish time gives a clean 1..16 that needs no second table, and a value
 * ESP-IDF adds past the end of out_bacnet's name list reads as state 1,
 * "Unknown", rather than as an unnamed state. */
static uint16_t restart_reason(void)
{
    return (uint16_t)((unsigned)esp_reset_reason() + 1u);
}

/* Days since the battery was last changed, from our stored date if one is
 * set, else from what the UPS reports. Needs a real clock: without a network
 * this board has none yet, so it answers "not reported" rather than guessing
 * from an epoch that starts at boot. The PCF85063 on this board could fix
 * that later; nothing drives it today. */
static uint16_t battery_age_days(const char *ups_date)
{
    const char *d = s_bat_date[0] ? s_bat_date : ups_date;
    if (!d || !d[0]) return OUT_ABSENT;

    struct tm bt = {0};
    if (sscanf(d, "%4d-%2d-%2d", &bt.tm_year, &bt.tm_mon, &bt.tm_mday) != 3) return OUT_ABSENT;
    bt.tm_year -= 1900; bt.tm_mon -= 1; bt.tm_hour = 12;

    time_t now = time(NULL);
    struct tm nt;
    if (!gmtime_r(&now, &nt) || nt.tm_year + 1900 < 2024) return OUT_ABSENT;  /* clock unset */

    time_t then = mktime(&bt);
    if (then <= 0 || now <= then) return 0;
    long days = (long)((now - then) / 86400);
    return days > (long)OUT_REG_MAX_REAL ? OUT_REG_MAX_REAL : (uint16_t)days;
}

/* "v0.20.2-dirty" -> 20*100 + 2. Best effort: 0 when it will not parse. */
static uint16_t fw_version(void)
{
    const esp_app_desc_t *a = esp_app_get_description();
    if (!a) return 0;
    const char *p = a->version;
    while (*p && (*p < '0' || *p > '9')) p++;        /* skip a leading 'v' */
    int maj = 0, min = 0;
    if (sscanf(p, "%d.%d", &maj, &min) < 1) return 0;
    long v = (long)maj * 100 + min;
    return v > (long)OUT_REG_MAX_REAL ? OUT_REG_MAX_REAL : (uint16_t)v;
}

/* ---- the build -------------------------------------------------------- */

static void build(out_points_t *p)
{
    ups_data_t d;
    ups_hid_get(&d);

    uint32_t nst = 0;
    bool have_nst = ups_hid_nut_bits(&nst);

    /* See STALE_US. `attached` alone is almost always enough; the time test
     * covers the path where it is not. */
    const bool ok = d.attached &&
                    (esp_timer_get_time() - d.last_update_us) < STALE_US;

    memset(p, 0, sizeof(*p));
    p->comm_ok = ok;

    /* Discretes. Every one reads false when the link is down: ups_hid's
     * invalidate_readings() has already set status to UPS_STATUS_UNKNOWN, and
     * `ok` gates the rest. A site must alarm on BOTH "on battery" and "comm
     * OK false" -- neither alone catches a dead adapter. */
    p->discrete[OUT_D_ONLINE]          = ok && (d.status & UPS_STATUS_ONLINE);
    p->discrete[OUT_D_ON_BATTERY]      = ok && (d.status & UPS_STATUS_ONBATT);
    p->discrete[OUT_D_LOW_BATTERY]     = ok && (d.status & UPS_STATUS_LOWBATT);
    p->discrete[OUT_D_REPLACE_BATTERY] = ok && (d.status & UPS_STATUS_REPLACEBATT);
    p->discrete[OUT_D_OVERLOAD]        = ok && (d.status & UPS_STATUS_OVERLOAD);
    p->discrete[OUT_D_COMM_OK]         = ok;
    p->discrete[OUT_D_CHARGING]        = ok && (d.status & UPS_STATUS_CHARGING);

    /* Boost, trim and "not protecting" are NOT in ups_data_t.status, which
     * carries seven bits. They come from upstream's full NUT token word. */
    if (ok && have_nst) {
        p->discrete[OUT_D_AVR_BOOST] = (nst & NST(NST_BOOST)) != 0;
        p->discrete[OUT_D_AVR_TRIM]  = (nst & NST(NST_TRIM))  != 0;
        p->discrete[OUT_D_OUTPUT_NOT_PROTECTING] =
            (nst & (NST(NST_OFF) | NST(NST_BYPASSAUTO) | NST(NST_BYPASSMAN))) != 0;
    }

    /* Measurements. */
    p->reg[OUT_R_CHARGE_PCT]   = meas(ok, d.battery_charge,  1.0f);
    p->reg[OUT_R_RUNTIME_S]    = meas(ok, d.battery_runtime, 1.0f);
    p->reg[OUT_R_LOAD_PCT]     = meas(ok, d.ups_load,        1.0f);
    p->reg[OUT_R_INPUT_V10]    = meas(ok, d.input_voltage,   10.0f);
    p->reg[OUT_R_OUTPUT_V10]   = meas(ok, d.output_voltage,  10.0f);
    p->reg[OUT_R_BATTERY_V10]  = meas(ok, d.battery_voltage, 10.0f);

    /* Register 6. Advanced by the caller; this only decides what a lost link
     * shows. OUT_HB_RUN keeps it counting so Niagara can tell "adapter alive,
     * UPS link dead" from "adapter dead". */
    p->reg[OUT_R_HEARTBEAT] = (!ok && s_hb == OUT_HB_INVALID) ? OUT_ABSENT : s_heartbeat;

    /* Register 7 IS SIX BITS WIDE and must stay that way. Discretes 6-10 are
     * deliberately not mirrored into it: widening it would change the number
     * a site already reads at a wired address, and anything reading only the
     * bitfield would see its value silently change meaning. The new discretes
     * are read at their own addresses. It also reads 0 on comm loss rather
     * than 65535, so a site watching only this register cannot see a phantom
     * "on battery" -- a deliberate departure from "registers read 65535",
     * carried over from the Python bridge. */
    uint16_t bits = 0;
    for (int i = 0; i <= OUT_D_COMM_OK; i++) if (p->discrete[i]) bits |= (uint16_t)(1u << i);
    p->reg[OUT_R_STATUS_BITS] = bits;

    p->reg[OUT_R_BATTERY_AGE_D] = ok ? battery_age_days(d.battery_mfr_date) : OUT_ABSENT;
    p->reg[OUT_R_TEST_RESULT]   = test_result(ok && (d.caps & UPS_CAP_TEST), d.test_result);

    /* Register 12. AudibleAlarmControl reads back 1/2/3; 0 means the UPS has
     * no such usage, which is "not reported" rather than a fourth state. */
    p->reg[OUT_R_BEEPER] = (ok && (d.caps & UPS_CAP_BEEPER) &&
                            d.beeper >= 1 && d.beeper <= 3)
                               ? (uint16_t)d.beeper : OUT_ABSENT;

    /* The AVR window: static for a unit, and what makes discretes 6 and 7
     * readable -- boost and trim say the mains left the window, these say
     * where it is. NUT variables rather than ups_data_t fields. */
    char v[24];
    p->reg[OUT_R_TRANSFER_LOW_V10] =
        (ok && ups_hid_var_find("input.transfer.low", v, sizeof(v)))
            ? meas(true, strtof(v, NULL), 10.0f) : OUT_ABSENT;
    p->reg[OUT_R_TRANSFER_HIGH_V10] =
        (ok && ups_hid_var_find("input.transfer.high", v, sizeof(v)))
            ? meas(true, strtof(v, NULL), 10.0f) : OUT_ABSENT;

    /* Discrete 9 is ours: the UPS cannot know its own service life. */
    uint16_t age = p->reg[OUT_R_BATTERY_AGE_D];
    p->discrete[OUT_D_BATTERY_AGE_ALARM] =
        s_bat_life_years > 0 && age != OUT_ABSENT &&
        age > (uint16_t)(s_bat_life_years * 365);

    /* The adapter's own facts: true whether or not the UPS is talking, which
     * is the point of giving them their own block. */
    p->reg[OUT_R_FW_VERSION]   = fw_version();
    p->reg[OUT_R_RESET_COUNT]  = s_boots;
    p->reg[OUT_R_RESET_REASON] = restart_reason();

    /* Which addresses exist at all. Everything not set here is absent from
     * the datastore and answers Modbus exception 02, NOT 65535. */
    for (int i = 0; i <= OUT_R_BEEPER; i++) p->reg_live[i] = true;
    p->reg_live[OUT_R_FW_VERSION]   = true;
    p->reg_live[OUT_R_RESET_COUNT]  = true;
    p->reg_live[OUT_R_RESET_REASON] = true;
    /* 12-15 and 19-23 stay false: reserved by the other track. Claiming one
     * without going through them is how the two halves of the fleet end up
     * meaning different things by the same address. */
}

static void tick(void *arg)
{
    (void)arg;
    /* Wraps 0..65534 so it never lands on 65535, which means "no data". */
    s_heartbeat = (uint16_t)((s_heartbeat + 1) % (OUT_REG_MAX_REAL + 1));

    out_points_t fresh;
    build(&fresh);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_pts = fresh;
    xSemaphoreGive(s_lock);
}

/* ---- settings and lifecycle ------------------------------------------- */

static void load_settings(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t n = sizeof(s_bat_date);
    if (nvs_get_str(h, KEY_BAT_DATE, s_bat_date, &n) != ESP_OK) s_bat_date[0] = '\0';
    int8_t life = 0;
    if (nvs_get_i8(h, KEY_BAT_LIFE, &life) == ESP_OK) s_bat_life_years = life;
    uint8_t hb = 0;
    if (nvs_get_u8(h, KEY_HB, &hb) == ESP_OK) s_hb = hb ? OUT_HB_RUN : OUT_HB_INVALID;
    /* KEY_PROTO holds the RS485 protocol. It has held other things:
     *
     *   - Before 2026-11-22 it was ONE protocol for the whole device, so it
     *     can contain 4 (Modbus TCP), which is not an RS485 protocol and
     *     becomes the TCP toggle instead. Discarding it would silently
     *     unconfigure a commissioned board.
     *   - 5 (BACnet/IP) is possible in principle and serves nothing now, so
     *     it loads as "nothing selected" -- see the case below.
     *
     * An older oc_bipen key may also still be in NVS from a brief period
     * when BACnet/IP was a separate toggle. It is deliberately not read:
     * nothing serves BACnet/IP, so honouring it could only produce the silent
     * dead state this function exists to avoid. It rots harmlessly. */
    uint8_t pr = 0;
    if (nvs_get_u8(h, KEY_PROTO, &pr) == ESP_OK) {
        switch (pr) {
        case OUT_PROTO_NONE:
        case OUT_PROTO_MODBUS_RTU:
        case OUT_PROTO_BACNET_MSTP:
            s_proto = (out_proto_t)pr;
            break;
        case OUT_PROTO_MODBUS_TCP:
            s_proto = OUT_PROTO_NONE;
            s_tcp_en = true;
            break;
        case OUT_PROTO_BACNET_IP:
            /* No output serves this now, so loading it would leave the board
             * silent while the page named a protocol -- the exact failure
             * every refusal in post_bus exists to prevent. It becomes
             * "nothing selected", which is visible and recoverable.
             *
             * No board should have it stored: it was refused at the API from
             * the day the value existed. This is for the one that somehow
             * does. */
            s_proto = OUT_PROTO_NONE;
            break;
        default:
            break;                  /* corrupt: keep the default */
        }
    }
    uint8_t te = 0;
    if (nvs_get_u8(h, KEY_TCPEN, &te) == ESP_OK) s_tcp_en = te != 0;
    uint8_t ad = 0;
    if (nvs_get_u8(h, KEY_ADDR, &ad) == ESP_OK && ad >= 1 && ad <= 127) s_addr = ad;
    int32_t bd = 0;
    if (nvs_get_i32(h, KEY_BAUD, &bd) == ESP_OK && bd >= 0) s_baud = bd;
    uint8_t cm = 0;
    if (nvs_get_u8(h, KEY_CMD, &cm) == ESP_OK) s_cmd_enabled = cm != 0;
    uint8_t dp = 0;
    if (nvs_get_u8(h, KEY_DEEP, &dp) == ESP_OK) s_deep_enabled = dp != 0;
    uint8_t fm = 0;
    if (nvs_get_u8(h, KEY_FMT, &fm) == ESP_OK &&
        fm >= OUT_FMT_8N1 && fm <= OUT_FMT_8N2) s_fmt = (out_fmt_t)fm;
    uint32_t dv = 0;
    if (nvs_get_u32(h, KEY_DEVID, &dv) == ESP_OK && dv <= OUT_DEVID_MAX) s_devid = dv;
    uint8_t ifr = 0;
    if (nvs_get_u8(h, KEY_INFOFR, &ifr) == ESP_OK && ifr >= 1) s_info_frames = ifr;
    uint8_t mm = 0;
    if (nvs_get_u8(h, KEY_MAXMAST, &mm) == ESP_OK && mm <= 127) s_max_master = mm;
    uint16_t mp = 0;
    if (nvs_get_u16(h, KEY_MBPORT, &mp) == ESP_OK && mp >= 1) s_mb_tcp_port = mp;
    uint8_t tu = 0;
    if (nvs_get_u8(h, KEY_TCPUID, &tu) == ESP_OK && tu <= 247) s_tcp_uid = tu;
    nvs_close(h);
}

/* THE SET RETURN IS CHECKED, not just the commit.
 *
 * It used not to be, here and in every writer below, and the failure that
 * hides is nasty: nvs_set_*() refuses the write (no free entry, or a key
 * stored once with a different type), nvs_commit() cheerfully succeeds
 * because there is nothing to flush, the setter answers ESP_OK, the caller
 * updates its RAM copy -- and the board behaves correctly until it reboots,
 * when the setting is simply back to what it was. A site would have had to
 * notice that a saved port did not survive a power cycle.
 *
 * It also makes webui's "could not be stored" message reachable for the right
 * reason instead of being dead code. */
static esp_err_t put_u8(const char *key, uint8_t v)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    e = nvs_set_u8(h, key, v);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}

out_proto_t out_common_proto(void) { return s_proto; }

esp_err_t out_common_set_proto(out_proto_t p)
{
    /* RS485, or nothing. Modbus TCP is not a value here -- it is its own
     * switch, because it runs alongside these rather than instead of them.
     *
     * BACnet/IP is not a value either. Decided 2026-11-22: out of scope for
     * this board, which exists to put a USB UPS on an RS485 trunk. The enum
     * member stays for out_common_proto_name() and for the NVS migration
     * path, but nothing can store it. */
    if (p != OUT_PROTO_NONE && p != OUT_PROTO_MODBUS_RTU &&
        p != OUT_PROTO_BACNET_MSTP) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t e = put_u8(KEY_PROTO, (uint8_t)p);
    if (e == ESP_OK) {
        s_proto = p;
        /* Deliberately not started or stopped here. An output that is holding
         * a UART or a socket cannot be swapped under a live trunk safely, and
         * a BMS seeing a device vanish mid-poll logs a fault. The page asks
         * for a restart, which is one clean transition instead of two. */
        ESP_LOGW(TAG, "protocol set to %s; restart to apply", out_common_proto_name(p));
    }
    return e;
}

const char *out_common_proto_name(out_proto_t p)
{
    switch (p) {
    case OUT_PROTO_MODBUS_RTU:  return "Modbus RTU";
    case OUT_PROTO_BACNET_MSTP: return "BACnet MS/TP";
    case OUT_PROTO_MODBUS_TCP:  return "Modbus TCP";
    case OUT_PROTO_BACNET_IP:   return "BACnet/IP";
    default:                    return "none";
    }
}

int out_common_address(void) { return s_addr; }

bool out_common_address_valid(int addr, out_proto_t p)
{
    switch (p) {
    case OUT_PROTO_BACNET_MSTP:
        /* MS/TP token passing cannot address above 127 (ASHRAE 135, Nmax
         * master), and 0 is a legal master address. */
        return addr >= 0 && addr <= 127;
    case OUT_PROTO_MODBUS_RTU:
        /* 1-247. 0 is the Modbus broadcast address and no device may own it;
         * 248-255 are reserved. Modbus TCP is absent from this switch on
         * purpose: its unit id is its own setting now and never judged here. */
        return addr >= 1 && addr <= 247;
    default:
        /* Nothing on the bus, or BACnet/IP where the address is an IP and
         * this field is unused. Accept the widest range so a stored value
         * survives a trip through "none". */
        return addr >= 0 && addr <= 247;
    }
}

esp_err_t out_common_set_address(int addr)
{
    /* Validated against the protocol in force. A caller changing BOTH must
     * check the pair itself before setting either, or the order decides
     * whether a legal combination is accepted -- see post_bus in webui. */
    /* Judged against the RS485 protocol, which is the only thing that uses
     * this number. Modbus TCP had its unit id carved out of here on
     * 2026-11-22 precisely so that one network's addressing could not
     * constrain the other's. */
    if (!out_common_address_valid(addr, s_proto)) return ESP_ERR_INVALID_ARG;
    esp_err_t e = put_u8(KEY_ADDR, (uint8_t)addr);
    if (e == ESP_OK) s_addr = addr;
    return e;
}

int out_common_baud(void) { return s_baud; }

esp_err_t out_common_set_baud(int baud)
{
    static const int ok[] = { OUT_BAUD_AUTO, 9600, 19200, 38400, 57600, 76800, 115200 };
    bool good = false;
    for (size_t i = 0; i < sizeof(ok) / sizeof(ok[0]); i++) if (ok[i] == baud) good = true;
    if (!good) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    e = nvs_set_i32(h, KEY_BAUD, baud);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    if (e == ESP_OK) s_baud = baud;
    return e;
}

/* Register 17. crashlog keeps a human-readable record but exposes no count,
 * so this is ours: one increment per boot, which is what the register says. */
static void bump_boots(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    uint16_t n = 0;
    nvs_get_u16(h, KEY_BOOTS, &n);
    if (n < OUT_REG_MAX_REAL) n++;
    /* Register 17 must not report a count that was not stored, or a board
     * with full settings memory would appear to reboot endlessly. */
    esp_err_t e = nvs_set_u16(h, KEY_BOOTS, n);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    if (e == ESP_OK) s_boots = n;
    else ESP_LOGW(TAG, "could not store the boot count: %s", esp_err_to_name(e));
}

esp_err_t out_common_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
    load_settings();
    bump_boots();
    build(&s_pts);

    const esp_timer_create_args_t a = { .callback = tick, .name = "out_common" };
    esp_err_t e = esp_timer_create(&a, &s_timer);
    if (e != ESP_OK) return e;
    e = esp_timer_start_periodic(s_timer, REFRESH_US);
    if (e != ESP_OK) return e;

    ESP_LOGI(TAG, "point set up: boot %u, battery life %d years, heartbeat %s on comm loss",
             (unsigned)s_boots, s_bat_life_years,
             s_hb == OUT_HB_RUN ? "keeps counting" : "reads 65535");
    /* All three outputs, because all three can be on. A log line naming only
     * one of them is how "but it IS configured" arguments start. */
    ESP_LOGI(TAG, "bus: %s at address %d, baud %s, %s; Modbus TCP %s",
             out_common_proto_name(s_proto), s_addr,
             s_baud == OUT_BAUD_AUTO ? "auto" : "fixed",
             out_common_format_name(s_fmt),
             s_tcp_en ? "on" : "off");
    if (s_tcp_en) {
        ESP_LOGI(TAG, "Modbus TCP: port %d, unit id %s", s_mb_tcp_port,
                 s_tcp_uid == OUT_TCP_UID_ANY ? "any" : "filtered");
    }
    return ESP_OK;
}

void out_common_get(out_points_t *out)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_pts;
    xSemaphoreGive(s_lock);
}

esp_err_t out_common_set_battery_date(const char *yyyy_mm_dd)
{
    int y, m, dd;
    bool clear = !yyyy_mm_dd || !yyyy_mm_dd[0];
    if (!clear && (sscanf(yyyy_mm_dd, "%4d-%2d-%2d", &y, &m, &dd) != 3 ||
                   y < 1990 || y > 2200 || m < 1 || m > 12 || dd < 1 || dd > 31)) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    if (clear) {
        /* A key that was never there is not a failure. */
        e = nvs_erase_key(h, KEY_BAT_DATE);
        if (e == ESP_ERR_NVS_NOT_FOUND) e = ESP_OK;
        if (e == ESP_OK) s_bat_date[0] = '\0';
    } else {
        e = nvs_set_str(h, KEY_BAT_DATE, yyyy_mm_dd);
        if (e == ESP_OK) strlcpy(s_bat_date, yyyy_mm_dd, sizeof(s_bat_date));
    }
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}

void out_common_get_battery_date(char *out, size_t cap)
{
    strlcpy(out, s_bat_date, cap);
}

esp_err_t out_common_set_battery_life_years(int years)
{
    if (years < 0 || years > 100) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    e = nvs_set_i8(h, KEY_BAT_LIFE, (int8_t)years);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    if (e == ESP_OK) s_bat_life_years = years;
    return e;
}

esp_err_t out_common_set_hb_policy(out_hb_policy_t p)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    e = nvs_set_u8(h, KEY_HB, p == OUT_HB_RUN ? 1 : 0);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    if (e == ESP_OK) s_hb = p;
    return e;
}

out_hb_policy_t out_common_hb_policy(void) { return s_hb; }

uint32_t out_common_device_id(void) { return s_devid; }

esp_err_t out_common_set_device_id(uint32_t id)
{
    if (id > OUT_DEVID_MAX) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    e = nvs_set_u32(h, KEY_DEVID, id);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    if (e == ESP_OK) s_devid = id;
    return e;
}

int out_common_info_frames(void) { return s_info_frames; }

esp_err_t out_common_set_info_frames(int n)
{
    if (n < 1 || n > 255) return ESP_ERR_INVALID_ARG;
    esp_err_t e = put_u8(KEY_INFOFR, (uint8_t)n);
    if (e == ESP_OK) s_info_frames = n;
    return e;
}

int out_common_max_master(void) { return s_max_master; }

esp_err_t out_common_set_max_master(int n)
{
    if (n < 0 || n > 127) return ESP_ERR_INVALID_ARG;
    esp_err_t e = put_u8(KEY_MAXMAST, (uint8_t)n);
    if (e == ESP_OK) s_max_master = n;
    return e;
}

/* u16 helper: the ports are 16-bit. Checks the set, as put_u8 explains. */
static esp_err_t put_u16(const char *key, uint16_t v)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    e = nvs_set_u16(h, key, v);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}

int out_common_tcp_uid(void) { return s_tcp_uid; }

esp_err_t out_common_set_tcp_uid(int uid)
{
    /* 0 is the wildcard, 1-247 a filter. 248-255 are reserved by the spec and
     * 255 is answered unconditionally by the stack anyway, so there is nothing
     * a value above 247 could usefully mean. */
    if (uid < 0 || uid > 247) return ESP_ERR_INVALID_ARG;
    esp_err_t e = put_u8(KEY_TCPUID, (uint8_t)uid);
    if (e == ESP_OK) s_tcp_uid = uid;
    return e;
}

int out_common_mb_tcp_port(void) { return s_mb_tcp_port; }

esp_err_t out_common_set_mb_tcp_port(int port)
{
    if (port < 1 || port > 65535) return ESP_ERR_INVALID_ARG;
    esp_err_t e = put_u16(KEY_MBPORT, (uint16_t)port);
    if (e == ESP_OK) s_mb_tcp_port = port;
    return e;
}

bool out_common_tcp_enabled(void) { return s_tcp_en; }

esp_err_t out_common_set_tcp_enabled(bool on)
{
    esp_err_t e = put_u8(KEY_TCPEN, on ? 1 : 0);
    if (e == ESP_OK) s_tcp_en = on;
    return e;
}

bool out_common_cmd_enabled(void) { return s_cmd_enabled; }

bool out_common_deep_enabled(void) { return s_deep_enabled; }

esp_err_t out_common_set_deep_enabled(bool on)
{
    esp_err_t e = put_u8(KEY_DEEP, on ? 1 : 0);
    if (e == ESP_OK) {
        s_deep_enabled = on;
        ESP_LOGW(TAG, "DEEP battery tests from the bus are now %s",
                 on ? "ENABLED" : "off");
    }
    return e;
}

/* A stuck or misconfigured master must not be able to cycle the battery. A
 * quick test is seconds of load on battery; one every half hour is generous
 * for any real use and useless as a way to flatten a pack.
 *
 * A deep test is a different animal and gets a different number. It runs the
 * pack down to the low-battery threshold, so more than one a day is not
 * maintenance, it is wear -- and on a site where the mains is poor it is wear
 * with the load unprotected. */
#define CMD_MIN_GAP_US  (30LL * 60 * 1000000)
#define DEEP_MIN_GAP_US (24LL * 60 * 60 * 1000000)
static int64_t s_cmd_last_us;       /* any test start */
static int64_t s_deep_last_us;      /* deep starts only */

esp_err_t out_common_request_test(out_test_t which, const char *via)
{
    const bool deep = (which == OUT_TEST_DEEP);
    const bool stop = (which == OUT_TEST_STOP);
    const char *what = deep ? "DEEP battery test (runtime calibration)"
                     : stop ? "stop battery test"
                            : "quick battery self-test";

    /* Reversing "read-only on the bus" was Tobias's call on 2026-11-22; the
     * reasons it was the rule are unchanged, so these guards are the
     * compensation: the board owner opts in, the UPS must say it can
     * self-test, and the rate limit means the worst a hostile or broken
     * master achieves is one quick test every half hour. */
    /* Each test answers to its own switch; stop answers to either, because a
     * board that accepts any command must be able to take it back. */
    const bool allowed = deep ? s_deep_enabled
                       : stop ? (s_cmd_enabled || s_deep_enabled)
                              : s_cmd_enabled;
    if (!allowed) {
        ESP_LOGW(TAG, "%s requested via %s, but %s off on this board", what, via,
                 deep ? "deep tests are" : "commands are");
        return ESP_ERR_NOT_ALLOWED;
    }
    ups_data_t d;
    ups_hid_get(&d);
    if (!d.attached) {
        ESP_LOGW(TAG, "%s requested via %s, but no UPS is attached", what, via);
        return ESP_ERR_NOT_FOUND;
    }
    if (!(d.caps & UPS_CAP_TEST)) {
        /* From the UPS's own report descriptor, never from its model name.
         * One capability covers start and abort alike. */
        ESP_LOGW(TAG, "%s requested via %s, but this UPS does not report that it can",
                 what, via);
        return ESP_ERR_NOT_SUPPORTED;
    }

    const int64_t now = esp_timer_get_time();
    if (!stop) {
        if (s_cmd_last_us && (now - s_cmd_last_us) < CMD_MIN_GAP_US) {
            ESP_LOGW(TAG, "%s requested via %s %llds after the last test; ignored "
                          "(minimum %lld min between tests)", what, via,
                     (long long)((now - s_cmd_last_us) / 1000000),
                     (long long)(CMD_MIN_GAP_US / 60000000));
            return ESP_ERR_INVALID_STATE;
        }
        if (deep && s_deep_last_us && (now - s_deep_last_us) < DEEP_MIN_GAP_US) {
            ESP_LOGW(TAG, "%s requested via %s %lldh after the last deep test; ignored "
                          "(minimum %lld h)", what, via,
                     (long long)((now - s_deep_last_us) / 3600000000LL),
                     (long long)(DEEP_MIN_GAP_US / 3600000000LL));
            return ESP_ERR_INVALID_STATE;
        }
        s_cmd_last_us = now;
        if (deep) s_deep_last_us = now;
    }

    const esp_err_t e = ups_hid_command(
        deep ? UPS_CMD_TEST_DEEP : stop ? UPS_CMD_TEST_ABORT : UPS_CMD_TEST_QUICK, 0);
    if (stop) {
        ESP_LOGW(TAG, "BATTERY TEST STOPPED via %s: %s", via, esp_err_to_name(e));
    } else {
        ESP_LOGW(TAG, "%s started via %s: %s. The load is on battery while it runs%s",
                 what, via, esp_err_to_name(e),
                 deep ? ", which for a calibration means until the battery is low." : ".");
    }
    return ESP_OK;
}

esp_err_t out_common_set_cmd_enabled(bool on)
{
    esp_err_t e = put_u8(KEY_CMD, on ? 1 : 0);
    if (e == ESP_OK) {
        s_cmd_enabled = on;
        ESP_LOGW(TAG, "commands from the bus are now %s", on ? "ENABLED" : "off");
    }
    return e;
}

out_fmt_t out_common_format(void) { return s_fmt; }

esp_err_t out_common_set_format(out_fmt_t f)
{
    if (f < OUT_FMT_8N1 || f > OUT_FMT_8N2) return ESP_ERR_INVALID_ARG;
    esp_err_t e = put_u8(KEY_FMT, (uint8_t)f);
    if (e == ESP_OK) s_fmt = f;
    return e;
}

const char *out_common_format_name(out_fmt_t f)
{
    switch (f) {
    case OUT_FMT_8E1: return "8E1";
    case OUT_FMT_8O1: return "8O1";
    case OUT_FMT_8N2: return "8N2";
    default:          return "8N1";
    }
}

const char *out_common_discrete_name(int i)
{
    static const char *const N[OUT_DISCRETE_N] = {
        "Online", "On battery", "Low battery", "Replace battery", "Overload",
        "UPS comm OK", "AVR boost", "AVR trim", "Charging",
        "Battery age alarm", "Output not protecting",
    };
    return (i >= 0 && i < OUT_DISCRETE_N) ? N[i] : "";
}

const char *out_common_reg_name(int i)
{
    switch (i) {
    case OUT_R_CHARGE_PCT:       return "Battery charge";
    case OUT_R_RUNTIME_S:        return "Runtime remaining";
    case OUT_R_LOAD_PCT:         return "Load";
    case OUT_R_INPUT_V10:        return "Input voltage";
    case OUT_R_OUTPUT_V10:       return "Output voltage";
    case OUT_R_BATTERY_V10:      return "Battery voltage";
    case OUT_R_HEARTBEAT:        return "Heartbeat";
    case OUT_R_STATUS_BITS:      return "Status bitfield (bits 0-5)";
    case OUT_R_BATTERY_AGE_D:    return "Days since battery change";
    case OUT_R_TEST_RESULT:      return "Self-test result";
    case OUT_R_BEEPER:           return "Beeper";
    case OUT_R_TRANSFER_LOW_V10: return "Input transfer low";
    case OUT_R_TRANSFER_HIGH_V10:return "Input transfer high";
    case OUT_R_FW_VERSION:       return "Firmware version";
    case OUT_R_RESET_COUNT:      return "Restart counter";
    case OUT_R_RESET_REASON:     return "Last restart reason";
    default:                     return "";     /* reserved: no such address */
    }
}

const char *out_common_reg_unit(int i)
{
    switch (i) {
    case OUT_R_CHARGE_PCT:
    case OUT_R_LOAD_PCT:         return "%";
    case OUT_R_RUNTIME_S:        return "s";
    case OUT_R_INPUT_V10:
    case OUT_R_OUTPUT_V10:
    case OUT_R_BATTERY_V10:
    case OUT_R_TRANSFER_LOW_V10:
    case OUT_R_TRANSFER_HIGH_V10:return "V x10";
    case OUT_R_BATTERY_AGE_D:    return "days";
    default:                     return "";
    }
}
