/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * BACnet MS/TP output: the same point set out_modbus serves, as BACnet
 * objects on the RS485 trunk.
 *
 * OBJECT NUMBERING. Instance = the Modbus address + 1, for every point:
 *
 *     BI1..BI11  <- discretes 0..10
 *     AI1..AI19  <- registers 0..18, minus the ones that make no sense here
 *     MSI10/MSI19 <- registers 9 and 18, which are enumerations rather than
 *                    measurements, so they are Multi-State Inputs with named
 *                    states instead of floats a person has to decode
 *
 * This deliberately departs from the "BACnet (proposed)" table in the project
 * brief, which had BI1 = On battery and BI2 = Online -- the reverse of the
 * Modbus discretes. One ordering for both protocols is worth more than
 * matching a proposal that was marked as one, because the alternative is a
 * commissioning engineer holding two different maps for the same twelve
 * facts. The +1 is because BACnet instances conventionally start at 1 and
 * Modbus addresses at 0; it is the only arithmetic between the two maps.
 *
 * WHAT BACNET DOES BETTER THAN MODBUS, AND WHERE WE USE IT.
 *
 * Modbus has no way to say "I do not know", which is why registers read 65535
 * and the register map carries three pages of convention about it. BACnet has
 * Reliability, so a point whose value is unavailable says
 * COMMUNICATION_FAILURE and keeps its last value out of the way, instead of
 * publishing a number that a trend log would happily plot. We use it:
 *
 *   - UPS link lost, or the UPS does not report that point -> every affected
 *     object reads COMMUNICATION_FAILURE
 *   - "Comm OK" (BI6) is the one object that stays reliable either way: it is
 *     the thing being reported, so it must never be unreadable
 *
 * Real units, too: Modbus register 3 is volts x10 because it has to be an
 * integer. AI4 is volts, because an AI is a float with a units property.
 *
 * READ-ONLY, WITH ONE EXCEPTION: BV1.
 *
 * Every BI and AI here is read-only, and a WriteProperty to one earns
 * write-access-denied. BV1 is the single writable point on the device, and
 * writing it ACTIVE starts one quick battery self-test -- the BACnet half of
 * what Modbus does with coil 100.
 *
 * It is a Binary VALUE, not a Binary Output, because a BO models a physical
 * output this board does not have. bo.c is still not compiled in.
 *
 * Nor is BV1 commandable: BACNET_OBJECT_BINARY_VALUE_COMMANDABLE is left
 * undefined in the vendored stack, so present-value is a plain bool with no
 * priority array. That is what lets this be a momentary trigger -- the write
 * lands, we start the test, and we put present-value straight back to
 * INACTIVE. A commandable BV would have fought that, because the value would
 * have belonged to whatever priority wrote it.
 *
 * BACNET REFUSES BETTER THAN MODBUS, and we use that. Coil 100 is always
 * writable; when commands are off for a board, esp-modbus accepts the write
 * and we drop it with a log line the technician never sees. BV1 instead gets
 * Binary_Value_Write_Disable(), and bv.c then answers the write itself with
 * write-access-denied -- a refusal the BMS can see.
 *
 * (bv.c also drops present-value from the object's writable-property list
 * when writes are disabled, which sounds better still, but is worth nothing
 * HERE: the only thing in this stack that reads that list is the
 * backup-and-restore configuration encoder, behind BACNET_BACKUP_RESTORE,
 * which is not defined and whose bacfile.c is not vendored. In this build the
 * list answers nothing on the wire.)
 *
 * WHY WE WRAP bv.c's WRITE instead of using it directly: its own writable
 * list is wider than this device wants. object-name, description and
 * out-of-service are "always writable" there, regardless of Write_Enabled, so
 * a BMS could rename our object or park BV1 out of service -- and an
 * out-of-service BV1 keeps whatever present-value was last written to it,
 * which on a commissioning screen reads as a self-test that never ends.
 *
 * None of that could ever START a test: bv.c deliberately skips the write
 * callback while Out_Of_Service is set, which is what out-of-service means.
 * It is wrong for a device documented as read-only but for one trigger, not
 * unsafe. bv_write() below refuses every property except present-value, and
 * closes the out-of-service route as a side effect.
 */

#include "out_bacnet.h"
#include "out_common.h"
#include "board.h"

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <string.h>

static const char *TAG = "out_bac";

#if BOARD_HAS_RS485

#include "bacnet_port.h"

#include "bacnet/basic/binding/address.h"
#include "bacnet/basic/object/ai.h"
#include "bacnet/basic/object/bi.h"
#include "bacnet/basic/object/bv.h"
#include "bacnet/basic/object/ms-input.h"
#include "bacnet/basic/object/device.h"
#include "bacnet/basic/services.h"
#include "bacnet/basic/sys/mstimer.h"
#include "bacnet/basic/tsm/tsm.h"
#include "bacnet/datalink/datalink.h"
#include "bacnet/datalink/dlmstp.h"
/* dlmstp.h pulls in mstpdef.h, which is the wire constants only. The port
 * struct we have to fill in, and MSTP_Zero_Config_UUID_Init, are in mstp.h. */
#include "bacnet/datalink/mstp.h"
#include "bacnet/npdu.h"

/* Registers that exist in Modbus but have no business being an AI:
 *
 *   7  status bitfield -- every bit of it is already BI1..BI6. A float whose
 *      value is 0b101101 is not data, it is a Modbus workaround.
 */
/* The two registers that are ENUMERATIONS rather than measurements. BACnet
 * has an object type for exactly this, so they are Multi-State Inputs and a
 * BMS shows "Task watchdog" where an AI could only show 7.0.
 *
 * The instance rule does not change: still the Modbus address + 1. MSI and AI
 * are different object types, so MSI10 collides with nothing, and the point
 * list keeps one arithmetic rule for every object on the device.
 *
 * Nor do the VALUES change between protocols. An MSI present-value is 1..N by
 * ANSI/ASHRAE 135, and both these registers were made 1-based in out_common
 * for that reason, so a 4 on Modbus is state 4 here. */
static bool reg_is_enum(int addr)
{
    return addr == OUT_R_TEST_RESULT || addr == OUT_R_RESET_REASON;
}

static bool reg_is_bacnet_point(int addr)
{
    if (addr == OUT_R_STATUS_BITS) return false;
    if (reg_is_enum(addr)) return false;          /* served as an MSI below */
    return out_common_reg_name(addr)[0] != '\0';   /* "" = reserved address */
}

/* The scale and units an AI needs, from the unit string out_common already
 * publishes. Keeping this derived rather than duplicated is the point: adding
 * a register to out_common gives it a BACnet object with no edit here. */
static BACNET_ENGINEERING_UNITS reg_units(int addr, float *divisor)
{
    const char *u = out_common_reg_unit(addr);
    *divisor = 1.0f;
    if (!strcmp(u, "%"))     return UNITS_PERCENT;
    if (!strcmp(u, "s"))     return UNITS_SECONDS;
    if (!strcmp(u, "days"))  return UNITS_DAYS;
    if (!strcmp(u, "V x10")) { *divisor = 10.0f; return UNITS_VOLTS; }
    return UNITS_NO_UNITS;
}

/* The one writable object. BV1, not BV101 (coil 100 + 1): the "instance =
 * Modbus address + 1" rule above maps the POINT SET, and this is a command
 * rather than a point -- which is also why it is a different object type and
 * why Modbus put it at 100, far outside its data block. One BV on the device,
 * numbered 1, is what an engineer browsing it expects to find. */
#define SELFTEST_BV     1

/* STATE TEXT, in the format basic/sys/state_name.c parses: NUL-separated C
 * strings with an empty one at the end. Index + 1 is the state number, which
 * is also the register's own value.
 *
 * state_name_list_init() strdup()s each label, so these could have been
 * built at runtime -- they are literals because they never vary, and a
 * literal in flash costs no heap on a board that must not fragment it. */
static const char SELFTEST_STATES[] =
    "Passed\0"             /* 1 */
    "Done, warning\0"      /* 2 */
    "Failed\0"             /* 3 */
    "In progress\0"        /* 4 */
    "Aborted\0"            /* 5 */
    "Scheduled\0"          /* 6  the other track's; we never emit it */
    "Unknown\0"            /* 7  moved off 0 so every enum starts at 1 */
    "\0";

/* esp_reset_reason_t order, shifted by one in out_common's restart_reason().
 * The enum is append-only upstream, so this list IS the table -- keep it in
 * step with esp_system.h and nothing else needs touching. */
static const char RESTART_STATES[] =
    "Unknown\0"            /*  1  ESP_RST_UNKNOWN    */
    "Power-on\0"           /*  2  ESP_RST_POWERON    */
    "External pin\0"       /*  3  ESP_RST_EXT        */
    "Software restart\0"   /*  4  ESP_RST_SW         */
    "Panic\0"              /*  5  ESP_RST_PANIC      */
    "Interrupt watchdog\0" /*  6  ESP_RST_INT_WDT    */
    "Task watchdog\0"      /*  7  ESP_RST_TASK_WDT   */
    "Other watchdog\0"     /*  8  ESP_RST_WDT        */
    "Deep sleep\0"         /*  9  ESP_RST_DEEPSLEEP  */
    "Brownout\0"           /* 10  ESP_RST_BROWNOUT   */
    "SDIO\0"               /* 11  ESP_RST_SDIO       */
    "USB peripheral\0"     /* 12  ESP_RST_USB        */
    "JTAG\0"               /* 13  ESP_RST_JTAG       */
    "eFuse error\0"        /* 14  ESP_RST_EFUSE      */
    "Power glitch\0"       /* 15  ESP_RST_PWR_GLITCH */
    "CPU lockup\0"         /* 16  ESP_RST_CPU_LOCKUP */
    "\0";

/* What each binary SAYS, instead of "Active" and "Inactive". Free -- bi.c
 * already carries both properties -- and it is the difference between an
 * operator reading a point and decoding one. These are literals in flash, so
 * unlike the device object's name there is no lifetime question. */
static const struct { const char *on, *off; } BI_TEXT[OUT_DISCRETE_N] = {
    [OUT_D_ONLINE]                = { "Online",         "Not online"   },
    [OUT_D_ON_BATTERY]            = { "On battery",     "On mains"     },
    [OUT_D_LOW_BATTERY]           = { "Low",            "Normal"       },
    [OUT_D_REPLACE_BATTERY]       = { "Replace",        "OK"           },
    [OUT_D_OVERLOAD]              = { "Overloaded",     "Normal"       },
    [OUT_D_COMM_OK]               = { "OK",             "Lost"         },
    [OUT_D_AVR_BOOST]             = { "Boosting",       "Off"          },
    [OUT_D_AVR_TRIM]              = { "Trimming",       "Off"          },
    [OUT_D_CHARGING]              = { "Charging",       "Not charging" },
    [OUT_D_BATTERY_AGE_ALARM]     = { "Overdue",        "OK"           },
    [OUT_D_OUTPUT_NOT_PROTECTING] = { "Not protecting", "Protecting"   },
};

static uint8_t                       s_pdu[MAX_MPDU + 16];
/* Set by the write callback, acted on by bac_task. NOT done inside the
 * callback: it runs from npdu_handler(), before the SimpleAck is encoded, and
 * a USB HID command in there would push the ack past what a master waits
 * for. Same shape as out_modbus, where the coil is written by the stack and
 * read by the task loop. */
static volatile bool                 s_bv_written;
static volatile struct mstp_port_struct_t s_mstp;
static struct dlmstp_user_data_t     s_mstp_user;
static uint8_t                       s_in[DLMSTP_MPDU_MAX];
static uint8_t                       s_out[DLMSTP_MPDU_MAX];
static out_bacnet_stat_t             s_stat;
static SemaphoreHandle_t             s_stat_lock;

/* Present-value is the only thing on BV1 a BMS may write. bv.c would also
 * accept object-name, description and out-of-service, which this device has
 * no business letting a trunk change -- see the header comment. Refusing them
 * here is one `if`, and it keeps "read-only but for the trigger" true as
 * written rather than true with footnotes. */
static bool bv_write(BACNET_WRITE_PROPERTY_DATA *wp_data)
{
    if (wp_data->object_property != PROP_PRESENT_VALUE) {
        wp_data->error_class = ERROR_CLASS_PROPERTY;
        wp_data->error_code  = ERROR_CODE_WRITE_ACCESS_DENIED;
        return false;
    }
    /* And this still refuses it when the board has commands switched off. */
    return Binary_Value_Write_Property(wp_data);
}

static struct dlmstp_rs485_driver s_driver = {
    .init                 = bacnet_rs485_init,
    .send                 = bacnet_rs485_send,
    .read                 = bacnet_rs485_read,
    .transmitting         = bacnet_rs485_transmitting,
    .baud_rate            = bacnet_rs485_baud,
    .baud_rate_set        = bacnet_rs485_baud_set,
    .silence_milliseconds = bacnet_rs485_silence_ms,
    .silence_reset        = bacnet_rs485_silence_reset,
};

static object_functions_t s_objects[] = {
    { OBJECT_DEVICE, NULL, Device_Count, Device_Index_To_Instance,
      Device_Valid_Object_Instance_Number, Device_Object_Name,
      Device_Read_Property_Local, Device_Write_Property_Local,
      Device_Property_Lists, DeviceGetRRInfo, NULL, NULL, NULL, NULL, NULL,
      NULL, NULL, NULL, NULL, NULL, Device_Writable_Property_List },

    { OBJECT_BINARY_INPUT, Binary_Input_Init, Binary_Input_Count,
      Binary_Input_Index_To_Instance, Binary_Input_Valid_Instance,
      Binary_Input_Object_Name, Binary_Input_Read_Property, NULL,
      Binary_Input_Property_Lists, NULL, NULL, Binary_Input_Encode_Value_List,
      Binary_Input_Change_Of_Value, Binary_Input_Change_Of_Value_Clear, NULL,
      NULL, NULL, Binary_Input_Create, Binary_Input_Delete, NULL,
      Binary_Input_Writable_Property_List },

    { OBJECT_ANALOG_INPUT, Analog_Input_Init, Analog_Input_Count,
      Analog_Input_Index_To_Instance, Analog_Input_Valid_Instance,
      Analog_Input_Object_Name, Analog_Input_Read_Property, NULL,
      Analog_Input_Property_Lists, NULL, NULL, Analog_Input_Encode_Value_List,
      Analog_Input_Change_Of_Value, Analog_Input_Change_Of_Value_Clear,
      Analog_Input_Intrinsic_Reporting, NULL, NULL, Analog_Input_Create,
      Analog_Input_Delete, NULL, Analog_Input_Writable_Property_List },

    { OBJECT_MULTI_STATE_INPUT, Multistate_Input_Init, Multistate_Input_Count,
      Multistate_Input_Index_To_Instance, Multistate_Input_Valid_Instance,
      Multistate_Input_Object_Name, Multistate_Input_Read_Property, NULL,
      Multistate_Input_Property_Lists, NULL, NULL,
      Multistate_Input_Encode_Value_List, Multistate_Input_Change_Of_Value,
      Multistate_Input_Change_Of_Value_Clear, NULL, NULL, NULL,
      Multistate_Input_Create, Multistate_Input_Delete, NULL,
      Multistate_Input_Writable_Property_List },

    /* The only row with a write function, and it is ours rather than bv.c's
     * -- see bv_write(). Present-value still ends up in
     * Binary_Value_Write_Property, which does the Write_Enabled check, so
     * "commands are off for this board" is answered by the stack. */
    { OBJECT_BINARY_VALUE, Binary_Value_Init, Binary_Value_Count,
      Binary_Value_Index_To_Instance, Binary_Value_Valid_Instance,
      Binary_Value_Object_Name, Binary_Value_Read_Property,
      bv_write, Binary_Value_Property_Lists, NULL, NULL,
      Binary_Value_Encode_Value_List, Binary_Value_Change_Of_Value,
      Binary_Value_Change_Of_Value_Clear, NULL, NULL, NULL,
      Binary_Value_Create, Binary_Value_Delete, NULL,
      Binary_Value_Writable_Property_List },

    { MAX_BACNET_OBJECT_TYPE, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
      NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL }
};

/* The device instance, and the name a BMS discovers us by.
 *
 * Derived from the MAC so twenty adapters on twenty trunks do not all claim
 * instance 12345, which is the sample value in every vendor's example code
 * and therefore the one most likely to collide. 22 bits because 4194303 is
 * the largest legal instance. */
static uint32_t device_instance_from_mac(char *name, size_t cap)
{
    uint8_t mac[6] = { 0 };
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(name, cap, "UPS-%02X%02X", mac[4], mac[5]);

    /* The configured instance wins; 0 means derive. The object NAME is always
     * derived, because it identifies the hardware rather than its place in a
     * numbering plan -- two adapters given the same instance by mistake are
     * much easier to tell apart if their names still differ. */
    const uint32_t configured = out_common_device_id();
    if (configured != OUT_DEVID_DERIVE) return configured;
    return ((uint32_t)mac[3] << 16 | (uint32_t)mac[4] << 8 | mac[5]) & OUT_DEVID_MAX;
}

/* bv.c calls this after a present-value write it has already accepted, which
 * means Write_Enabled was true -- so there is no permission check to repeat
 * here. Only ACTIVE starts anything: a master writing INACTIVE is clearing
 * the point, which is what we do ourselves anyway. */
static void bv_written(uint32_t instance, BACNET_BINARY_PV old_value,
                       BACNET_BINARY_PV value)
{
    (void)old_value;
    if (instance == SELFTEST_BV && value == BINARY_ACTIVE) s_bv_written = true;
}

/* Runs in bac_task, after the ack. Mirrors out_modbus's cmd_check(): clear
 * first so it is a trigger and not a state, then let out_common refuse it. */
static void cmd_check(void)
{
    if (!s_bv_written) return;
    s_bv_written = false;
    /* Back to INACTIVE whatever happens next, including a refusal -- the
     * object says "not running a test", not "a test was asked for". */
    Binary_Value_Present_Value_Set(SELFTEST_BV, BINARY_INACTIVE);

    if (out_common_request_selftest("BACnet BV1") != ESP_OK) return;

    xSemaphoreTake(s_stat_lock, portMAX_DELAY);
    s_stat.tests_started++;
    xSemaphoreGive(s_stat_lock);
}

static void create_objects(void)
{
    for (int i = 0; i < OUT_DISCRETE_N; i++) {
        const uint32_t inst = Binary_Input_Create((uint32_t)i + 1);
        Binary_Input_Name_Set(inst, out_common_discrete_name(i));
        Binary_Input_Description_Set(inst, out_common_discrete_name(i));
        Binary_Input_Present_Value_Set(inst, BINARY_INACTIVE);
        Binary_Input_Active_Text_Set(inst, BI_TEXT[i].on);
        Binary_Input_Inactive_Text_Set(inst, BI_TEXT[i].off);
    }

    /* The enumerated registers. Write-disabled explicitly: these are inputs,
     * and leaning on the object table's NULL write function alone would make
     * that depend on a default inside vendored code. */
    static const struct { int reg; const char *states; } MSI[] = {
        { OUT_R_TEST_RESULT,  SELFTEST_STATES },
        { OUT_R_RESET_REASON, RESTART_STATES  },
    };
    for (unsigned k = 0; k < sizeof MSI / sizeof MSI[0]; k++) {
        const uint32_t inst = Multistate_Input_Create((uint32_t)MSI[k].reg + 1);
        Multistate_Input_Name_Set(inst, out_common_reg_name(MSI[k].reg));
        Multistate_Input_Description_Set(inst, out_common_reg_name(MSI[k].reg));
        /* Sets Number_Of_States too: ms-input.c derives it from this list. */
        Multistate_Input_State_Text_List_Set(inst, MSI[k].states);
        Multistate_Input_Write_Disable(inst);
    }
    for (int a = 0; a < OUT_REG_N; a++) {
        if (!reg_is_bacnet_point(a)) continue;
        float div;
        const BACNET_ENGINEERING_UNITS u = reg_units(a, &div);
        const uint32_t inst = Analog_Input_Create((uint32_t)a + 1);
        Analog_Input_Name_Set(inst, out_common_reg_name(a));
        Analog_Input_Description_Set(inst, out_common_reg_name(a));
        Analog_Input_Units_Set(inst, u);
    }

    /* The self-test trigger. Created whether or not commands are enabled: a
     * point that disappears tells a BMS nothing, while a point that is there
     * and not writable tells it exactly where it stands -- and it keeps one
     * point list valid across every board. */
    const uint32_t bv = Binary_Value_Create(SELFTEST_BV);
    Binary_Value_Name_Set(bv, "Start battery self-test");
    Binary_Value_Description_Set(bv,
        "Write Active to start one quick battery self-test. The load runs on "
        "battery while it does. At most one every 30 minutes.");
    /* Worth setting: a BMS shows these instead of "Active"/"Inactive", and
     * "Start test" on a momentary point is the difference between an
     * operator knowing what a write does and guessing. */
    Binary_Value_Active_Text_Set(bv, "Start test");
    Binary_Value_Inactive_Text_Set(bv, "Idle");
    Binary_Value_Present_Value_Set(bv, BINARY_INACTIVE);
    Binary_Value_Write_Present_Value_Callback_Set(bv_written);
    /* Both branches explicitly. Binary_Value_Create() enables writes by
     * default, and leaning on that would make the opt-in depend on a default
     * inside vendored code. */
    if (out_common_cmd_enabled()) Binary_Value_Write_Enable(bv);
    else                          Binary_Value_Write_Disable(bv);
}

/* Copies the shared snapshot into the objects. Same cadence and same source
 * as out_modbus's refresh(), so the two protocols cannot disagree about what
 * the UPS said. */
static void refresh(void)
{
    out_points_t p;
    out_common_get(&p);

    for (int i = 0; i < OUT_DISCRETE_N; i++) {
        const uint32_t inst = (uint32_t)i + 1;
        Binary_Input_Present_Value_Set(
            inst, p.discrete[i] ? BINARY_ACTIVE : BINARY_INACTIVE);
        /* Comm OK is exempt: an object that reports whether the link is up
         * must stay readable when it is down, or it says nothing at all. */
        const bool exempt = (i == OUT_D_COMM_OK);
        Binary_Input_Reliability_Set(inst,
            (p.comm_ok || exempt) ? RELIABILITY_NO_FAULT_DETECTED
                                  : RELIABILITY_COMMUNICATION_FAILURE);
    }

    for (int a = 0; a < OUT_REG_N; a++) {
        if (!reg_is_bacnet_point(a)) continue;
        const uint32_t inst = (uint32_t)a + 1;
        float div;
        (void)reg_units(a, &div);

        /* OUT_ABSENT covers both "comm lost" and "this UPS does not report
         * it". BACnet does not need to tell those apart in the value, and
         * should not try: either way the number is not knowable, and
         * Reliability is the property that says so. */
        if (!p.reg_live[a] || p.reg[a] == OUT_ABSENT) {
            Analog_Input_Reliability_Set(inst, RELIABILITY_COMMUNICATION_FAILURE);
            continue;
        }
        Analog_Input_Present_Value_Set(inst, (float)p.reg[a] / div);
        Analog_Input_Reliability_Set(inst, RELIABILITY_NO_FAULT_DETECTED);
    }

    /* The enumerated registers, published as their own value: both protocols
     * number these the same, so there is no arithmetic here on purpose.
     *
     * 0 is not a state. Both encodings were made 1-based, so a zero means
     * something upstream is wrong rather than "state zero" -- and an MSI has
     * no way to say zero anyway. It reads as unavailable, like any other
     * number we cannot stand behind. */
    for (int a = 0; a < OUT_REG_N; a++) {
        if (!reg_is_enum(a)) continue;
        const uint32_t inst = (uint32_t)a + 1;
        if (!p.reg_live[a] || p.reg[a] == OUT_ABSENT || p.reg[a] == 0) {
            Multistate_Input_Reliability_Set(inst, RELIABILITY_COMMUNICATION_FAILURE);
            continue;
        }
        Multistate_Input_Present_Value_Set(inst, p.reg[a]);
        Multistate_Input_Reliability_Set(inst, RELIABILITY_NO_FAULT_DETECTED);
    }
}

static void bac_task(void *arg)
{
    (void)arg;
    struct mstimer refresh_timer;
    mstimer_set(&refresh_timer, 500);

    for (;;) {
        BACNET_ADDRESS src = { 0 };
        const uint16_t pdu_len = datalink_receive(&src, s_pdu, MAX_MPDU, 0);
        if (pdu_len) {
            npdu_handler(&src, s_pdu, pdu_len);
            xSemaphoreTake(s_stat_lock, portMAX_DELAY);
            s_stat.requests++;
            s_stat.last_request_us = esp_timer_get_time();
            xSemaphoreGive(s_stat_lock);
        }
        /* After npdu_handler, so the ack is already on its way out. */
        cmd_check();
        if (mstimer_expired(&refresh_timer)) {
            mstimer_reset(&refresh_timer);
            refresh();
            xSemaphoreTake(s_stat_lock, portMAX_DELAY);
            s_stat.bytes_tx = bacnet_rs485_bytes_tx();
            s_stat.bytes_rx = bacnet_rs485_bytes_rx();
            xSemaphoreGive(s_stat_lock);
        }
        tsm_timer_milliseconds(1);
        /* The MS/TP state machine is polled, and its timing budget at 76800
         * baud is tens of microseconds per byte. 1 ms is the shortest a
         * FreeRTOS delay can be at the default tick rate and still yield;
         * dlmstp's own receive loop does the byte-level work. */
        vTaskDelay(1);
    }
}

esp_err_t out_bacnet_start(void)
{
    if (out_common_proto() != OUT_PROTO_BACNET_MSTP) {
        return ESP_OK;                  /* not ours: cost the board nothing */
    }

    s_stat_lock = xSemaphoreCreateMutex();
    if (!s_stat_lock) return ESP_ERR_NO_MEM;

    int baud = out_common_baud();
    if (baud == OUT_BAUD_AUTO) {
        baud = 38400;
        ESP_LOGW(TAG, "baud is set to auto but autobaud is not built yet: using %d", baud);
    }
    const int mac = out_common_address();

    char dev_name[32];
    const uint32_t instance = device_instance_from_mac(dev_name, sizeof(dev_name));

    mstimer_init();
    address_init();
    Device_Init(s_objects);
    Device_Set_Object_Instance_Number(instance);
    Device_Object_Name_ANSI_Init(dev_name);
    create_objects();
    refresh();

    /* BRING THE UART UP OURSELVES, AND REFUSE TO GO ON WITHOUT IT.
     *
     * s_driver.init is set to bacnet_rs485_init, which looks like enough and
     * is not: nothing in bacnet-stack ever calls driver->init. Without this
     * call the UART is never configured, every read and send is a no-op, and
     * the master state machine -- which measures everything in line silence
     * -- finds every timeout permanently expired and transitions forever
     * inside dlmstp_receive(). That is a spin in our task, so the idle task
     * starves and the watchdog reboots the board. Selecting BACnet MS/TP took
     * a board off the network that way, every boot, until this was found.
     *
     * baud_set first: before the UART exists it only records the rate, and
     * bacnet_rs485_init() is what applies it through uart_param_config. */
    bacnet_rs485_baud_set((uint32_t)baud);
    bacnet_rs485_init();
    if (!bacnet_rs485_up()) {
        /* Fail closed. main() does not ESP_ERROR_CHECK this, so the board
         * keeps serving its page and somebody can read why the trunk is
         * silent -- which beats a reboot loop with no way in. */
        ESP_LOGE(TAG, "RS485 did not come up; not starting the MS/TP datalink");
        return ESP_ERR_INVALID_STATE;
    }

    const int info_frames = out_common_info_frames();
    const int max_master   = out_common_max_master();
    s_mstp.Nmax_info_frames = (uint8_t)info_frames;
    s_mstp.Nmax_master      = (uint8_t)max_master;
    s_mstp.InputBuffer      = s_in;
    s_mstp.InputBufferSize  = sizeof(s_in);
    s_mstp.OutputBuffer     = s_out;
    s_mstp.OutputBufferSize = sizeof(s_out);
    s_mstp.ZeroConfigEnabled = false;
    /* A master node, in the MS/TP sense: it takes part in token passing. That
     * is NOT the Modbus sense of master -- it still only answers requests and
     * never polls anyone. A slave node cannot hold the token, which rules out
     * most of what a BMS expects from a field device. */
    s_mstp.SlaveNodeEnabled  = false;
    s_mstp.CheckAutoBaud     = false;
    MSTP_Zero_Config_UUID_Init((struct mstp_port_struct_t *)&s_mstp);

    s_mstp_user.RS485_Driver = &s_driver;
    s_mstp.UserData = &s_mstp_user;

    if (!dlmstp_init((char *)&s_mstp)) {
        ESP_LOGE(TAG, "dlmstp_init failed");
        return ESP_FAIL;
    }
    dlmstp_set_mac_address((uint8_t)mac);
    dlmstp_set_baud_rate((uint32_t)baud);
    dlmstp_set_max_master((uint8_t)max_master);
    dlmstp_set_max_info_frames((uint8_t)info_frames);

    apdu_set_unrecognized_service_handler_handler(handler_unrecognized_service);
    apdu_set_unconfirmed_handler(SERVICE_UNCONFIRMED_WHO_IS, handler_who_is);
    apdu_set_unconfirmed_handler(SERVICE_UNCONFIRMED_WHO_HAS, handler_who_has);
    apdu_set_confirmed_handler(SERVICE_CONFIRMED_READ_PROPERTY, handler_read_property);
    apdu_set_confirmed_handler(SERVICE_CONFIRMED_READ_PROP_MULTIPLE,
                               handler_read_property_multiple);
    /* WriteProperty answers write-access-denied for every object except BV1,
     * which is the only one with a writable property list -- and for BV1 too
     * when commands are off for this board. Leaving the service unregistered
     * would answer "service not supported" instead, which reads to a BMS as a
     * broken device rather than a read-only one, and the brief's acceptance
     * test asks for the denial. */
    apdu_set_confirmed_handler(SERVICE_CONFIRMED_WRITE_PROPERTY, handler_write_property);
    apdu_set_confirmed_handler(SERVICE_CONFIRMED_DEVICE_COMMUNICATION_CONTROL,
                               handler_device_communication_control);

    /* Unsolicited I-Am so a BMS that was already listening finds us without
     * waiting for its next Who-Is sweep. */
    Send_I_Am(&Handler_Transmit_Buffer[0]);

    xSemaphoreTake(s_stat_lock, portMAX_DELAY);
    s_stat.running  = true;
    s_stat.mac      = mac;
    s_stat.baud     = baud;
    s_stat.instance = instance;
    s_stat.info_frames = info_frames;
    s_stat.max_master  = max_master;
    strlcpy(s_stat.device_name, dev_name, sizeof(s_stat.device_name));
    xSemaphoreGive(s_stat_lock);

    /* 8192: the MS/TP state machine plus a full APDU decode. The PDU buffer
     * is static rather than on this stack, but ReadPropertyMultiple decodes
     * into BACNET_APPLICATION_DATA_VALUE locals down a deep call chain, and
     * 4096 left no margin for a request we have never seen on hardware. A
     * stack overflow here is a panic, and a panic here is a board that drops
     * off the network -- the cheap side of that trade is obvious. */
    if (xTaskCreate(bac_task, "out_bac", 8192, NULL, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "BACnet MS/TP up: %s, device instance %lu%s, MAC %d, %d baud 8N1, "
                  "max master %d, %d info frame%s",
             dev_name, (unsigned long)instance,
             out_common_device_id() == OUT_DEVID_DERIVE ? " (derived)" : " (configured)",
             mac, baud, max_master, info_frames, info_frames == 1 ? "" : "s");
    ESP_LOGI(TAG, "BI1-BI%d from discretes 0-%d, AI from registers (+1), read-only; "
                  "BV%d self-test %s",
             OUT_DISCRETE_N, OUT_DISCRETE_N - 1, SELFTEST_BV,
             out_common_cmd_enabled() ? "WRITABLE" : "write-disabled");
    return ESP_OK;
}

void out_bacnet_get(out_bacnet_stat_t *out)
{
    if (!s_stat_lock) { memset(out, 0, sizeof(*out)); return; }
    xSemaphoreTake(s_stat_lock, portMAX_DELAY);
    *out = s_stat;
    xSemaphoreGive(s_stat_lock);
}

#else /* !BOARD_HAS_RS485 */

esp_err_t out_bacnet_start(void)
{
    if (out_common_proto() != OUT_PROTO_BACNET_MSTP) return ESP_OK;
    ESP_LOGE(TAG, "BACnet MS/TP selected, but this board has no RS485 transceiver");
    return ESP_ERR_NOT_SUPPORTED;
}

void out_bacnet_get(out_bacnet_stat_t *out) { memset(out, 0, sizeof(*out)); }

#endif /* BOARD_HAS_RS485 */
