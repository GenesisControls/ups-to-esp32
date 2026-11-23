/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * BACnet MS/TP output: the same point set out_modbus serves, as BACnet
 * objects on the RS485 trunk.
 *
 * OBJECT NUMBERING. Instance = the Modbus address + 1, for every point:
 *
 *     BI1..BI11  <- discretes 0..10
 *     AI1..AI19  <- registers 0..18, minus the ones that make no sense here
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
 * READ-ONLY. No writable objects at all: bo.c is not even compiled into the
 * vendored stack. The self-test trigger stays a Modbus coil for now -- giving
 * it a BACnet Binary Output would mean a writable object, a commandable
 * priority array and a second path into the same guard, and no site has asked.
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
static bool reg_is_bacnet_point(int addr)
{
    if (addr == OUT_R_STATUS_BITS) return false;
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

static uint8_t                       s_pdu[MAX_MPDU + 16];
static volatile struct mstp_port_struct_t s_mstp;
static struct dlmstp_user_data_t     s_mstp_user;
static uint8_t                       s_in[DLMSTP_MPDU_MAX];
static uint8_t                       s_out[DLMSTP_MPDU_MAX];
static out_bacnet_stat_t             s_stat;
static SemaphoreHandle_t             s_stat_lock;

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

static void create_objects(void)
{
    for (int i = 0; i < OUT_DISCRETE_N; i++) {
        const uint32_t inst = Binary_Input_Create((uint32_t)i + 1);
        Binary_Input_Name_Set(inst, out_common_discrete_name(i));
        Binary_Input_Description_Set(inst, out_common_discrete_name(i));
        Binary_Input_Present_Value_Set(inst, BINARY_INACTIVE);
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

    bacnet_rs485_baud_set((uint32_t)baud);

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
    /* WriteProperty is registered on purpose, and answers write-access-denied
     * for every object we expose, because none has a writable property list.
     * Leaving it unregistered would answer "service not supported" instead,
     * which reads to a BMS as a broken device rather than a read-only one --
     * and the brief's acceptance test asks for the denial. */
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

    /* 4096: the MS/TP state machine plus an APDU decode. MAX_APDU is 480 and
     * the PDU buffer above is static rather than on this stack for that
     * reason. */
    if (xTaskCreate(bac_task, "out_bac", 4096, NULL, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "BACnet MS/TP up: %s, device instance %lu%s, MAC %d, %d baud 8N1, "
                  "max master %d, %d info frame%s",
             dev_name, (unsigned long)instance,
             out_common_device_id() == OUT_DEVID_DERIVE ? " (derived)" : " (configured)",
             mac, baud, max_master, info_frames, info_frames == 1 ? "" : "s");
    ESP_LOGI(TAG, "BI1-BI%d from discretes 0-%d, AI from registers (+1), read-only",
             OUT_DISCRETE_N, OUT_DISCRETE_N - 1);
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
