# SPDX-License-Identifier: GPL-3.0-or-later
#
# Vendored bacnet-stack 1.7.0, MS/TP subset. See README-GENESIS.md for what is
# here, where it came from and under which licences.
#
# The source list is NOT a guess. It is bacnet-stack's own
# ports/esp32/extra_script.py, m5stamplc-mstp environment: its common group
# plus its MS/TP transport group, which is the upstream author's answer to
# "what does MS/TP on an ESP32 need". Two deliberate changes to that list:
#
#   - bo.c (Binary Output) is left OUT. A BO models a physical output this
#     board does not have, and in this stack it is commandable: a priority
#     array, a relinquish default and a second way to set the same value.
#   - bv.c (Binary Value) is ADDED, for the one writable point on the device:
#     BV1, which starts a battery self-test. It is NOT commandable here,
#     because BACNET_OBJECT_BINARY_VALUE_COMMANDABLE is not defined below --
#     so present-value is a plain bool with no priority array, which is what
#     lets out_bacnet treat a write as a momentary trigger and clear it.
#
# Device_Init and friends come from basic/server/bacnet_device.c. There is a
# SECOND implementation in basic/object/device.c, which the port does not
# compile -- adding it would be duplicate symbols at link time. Do not.

set(BACNET_SRCS
    # --- protocol encoding/decoding ---
    "src/bacnet/abort.c"
    "src/bacnet/bacaction.c"
    "src/bacnet/bacaddr.c"
    "src/bacnet/bacapp.c"
    "src/bacnet/bacdcode.c"
    "src/bacnet/bacdest.c"
    "src/bacnet/bacdevobjpropref.c"
    "src/bacnet/bacerror.c"
    "src/bacnet/bacint.c"
    "src/bacnet/bacreal.c"
    "src/bacnet/bacstr.c"
    "src/bacnet/cov.c"
    "src/bacnet/datetime.c"
    "src/bacnet/dcc.c"
    "src/bacnet/hostnport.c"
    "src/bacnet/iam.c"
    "src/bacnet/ihave.c"
    "src/bacnet/indtext.c"
    "src/bacnet/memcopy.c"
    "src/bacnet/npdu.c"
    "src/bacnet/proplist.c"
    "src/bacnet/rd.c"
    "src/bacnet/reject.c"
    "src/bacnet/rp.c"
    "src/bacnet/rpm.c"
    "src/bacnet/timestamp.c"
    "src/bacnet/whohas.c"
    "src/bacnet/whois.c"
    "src/bacnet/wp.c"

    # --- the device object and the two object types we expose ---
    "src/bacnet/basic/server/bacnet_device.c"
    "src/bacnet/basic/object/ai.c"
    "src/bacnet/basic/object/bi.c"
    "src/bacnet/basic/object/bv.c"

    # --- service handlers: everything a BMS asks an MS/TP device ---
    "src/bacnet/basic/service/h_apdu.c"
    "src/bacnet/basic/service/h_dcc.c"
    "src/bacnet/basic/service/h_noserv.c"
    "src/bacnet/basic/service/h_rd.c"
    "src/bacnet/basic/service/h_rp.c"
    "src/bacnet/basic/service/h_rpm.c"
    "src/bacnet/basic/service/h_whohas.c"
    "src/bacnet/basic/service/h_whois.c"
    "src/bacnet/basic/service/h_wp.c"
    "src/bacnet/basic/service/s_iam.c"
    "src/bacnet/basic/service/s_ihave.c"
    "src/bacnet/basic/npdu/h_npdu.c"

    # --- plumbing ---
    "src/bacnet/basic/binding/address.c"
    "src/bacnet/basic/datetime/datetime_mstimer.c"
    "src/bacnet/basic/tsm/tsm.c"
    "src/bacnet/basic/sys/days.c"
    "src/bacnet/basic/sys/dst.c"
    "src/bacnet/basic/sys/debug.c"
    "src/bacnet/basic/sys/fifo.c"
    "src/bacnet/basic/sys/keylist.c"
    "src/bacnet/basic/sys/mstimer.c"
    "src/bacnet/basic/sys/ringbuf.c"

    # --- MS/TP datalink ---
    "src/bacnet/datalink/cobs.c"
    "src/bacnet/datalink/crc.c"
    "src/bacnet/datalink/dlmstp.c"
    "src/bacnet/datalink/mstp.c"
    "src/bacnet/datalink/mstptext.c"
)

idf_component_register(
    SRCS ${BACNET_SRCS}
    INCLUDE_DIRS "src"
    REQUIRES esp_timer
)

# The stack is configured entirely by -D, exactly as the upstream ESP32 port's
# platformio.ini does it. PUBLIC because our out_bacnet includes these headers
# too and must see the same configuration -- a BACDL_MSTP mismatch between the
# stack and its caller changes struct layouts and fails at runtime, not build.
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    BACDL_MSTP                      # MS/TP is the only datalink compiled in
    MAX_APDU=480                    # the MS/TP value; BIP would use 1476
    MAX_TSM_TRANSACTIONS=1          # we answer requests, we do not issue them
    PRINT_ENABLED=0                 # no printf from the stack
    BACNET_BIG_ENDIAN=0             # ESP32-S3 is little-endian
    CRC_USE_TABLE                   # table-driven CRC: flash for speed
    BACAPP_MINIMAL                  # only the application datatypes we use
    BACAPP_TIMESTAMP
    BACNET_STACK_DEPRECATED_DISABLE
    BACNET_PROTOCOL_REVISION=16
    CONFIG_BACNET_BASIC_OBJECT_ANALOG_INPUT
    CONFIG_BACNET_BASIC_OBJECT_BINARY_INPUT
    CONFIG_BACNET_BASIC_OBJECT_BINARY_VALUE
    # NOT defined, on purpose: BACNET_OBJECT_BINARY_VALUE_COMMANDABLE, and
    # INTRINSIC_REPORTING / BINARY_VALUE_INTRINSIC_REPORTING. The first would
    # give BV1 a priority array it has no use for; the second is what guards
    # bv.c's only calls into bactext_*(), which is not compiled in here.
)

# Third-party code against ESP-IDF's -Werror=all. These are warnings in
# bacnet-stack, not in anything we wrote; patching 59 vendored files would
# make every future rebase a merge conflict. Our own components keep the
# full default warning set.
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=unused-but-set-variable
    -Wno-error=unused-variable
    -Wno-error=unused-function
    -Wno-error=type-limits
    -Wno-error=implicit-fallthrough
    -Wno-unused-parameter
    -Wno-missing-field-initializers
)
