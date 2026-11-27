/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Board abstraction for My UPS Pal.
 *
 * Rev A is the board that is made; Rev B (docs/rev-b.md) is its respin. Rev A
 * OLED is a second firmware IDENTITY on the same PCB, for a VCC-first display
 * module; it is not a second board. The two DEV_S3 targets are generic
 * ESP32-S3 boards people flash themselves (docs/dev-board.md): build from
 * source only, never released, never offered an update.
 *
 * Every pin the application touches is named here. Application code must not
 * contain GPIO numbers, so that a future board is a new block below and
 * nothing else.
 *
 * GPIO19/20 (USB D-/D+) are deliberately absent. They are fixed by silicon and
 * are owned by the usb_host peripheral, not by us.
 */
#ifndef UPSA_BOARD_H
#define UPSA_BOARD_H

#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "sdkconfig.h"

#if defined(CONFIG_UPSA_BOARD_REV_A)

#define BOARD_NAME              "My UPS Pal Rev A"
/* Picks the OTA asset: ups-adaptor-reva.bin. The two targets have different
 * pin maps, so installing the wrong one is not a cosmetic mistake. */
#define BOARD_OTA_TARGET        "reva"
/* What an image built FOR THIS BOARD calls itself in its app descriptor. The
 * OTA client refuses anything else, so a mislabelled asset cannot be
 * installed -- which has happened. */
#define BOARD_IMAGE_NAME        "ups-adaptor-reva"
/* Rev A: see docs/hardware.md pin assignment table. */
#define BOARD_PIN_VBUS_EN       GPIO_NUM_4   /* AP2171W EN, active high        */
/* Rev A uses an AP2171W: a SOT-23-5 load switch fits EN plus exactly one of
 * ILIM or FLAG, and this part chose FLAG. So fault reporting is back and the
 * current limit is fixed by the part rather than set by a resistor.
 * See docs/hardware-schematic.md section 1. */
#define BOARD_PIN_VBUS_FAULT    GPIO_NUM_5   /* AP2171W FLG, open drain, active low */
#define BOARD_PIN_LED_STATUS    GPIO_NUM_48  /* green, plain LED, 1k series    */
#define BOARD_PIN_LED_FAULT     GPIO_NUM_47  /* red, plain LED, 1k series      */
#define BOARD_PIN_I2C_SDA       GPIO_NUM_8
#define BOARD_PIN_I2C_SCL       GPIO_NUM_9
#define BOARD_PIN_FACTORY_BTN   GPIO_NUM_0   /* BOOT button, doubles as reset  */
#define BOARD_HAS_VBUS_SWITCH   1
#define BOARD_HAS_VBUS_FAULT    1
#define BOARD_HAS_STATUS_LEDS   1
/* A display on plain Rev A only with a VCC-first module; Kconfig hides every
 * other way, and this catches an edit to it. */
#if defined(CONFIG_UPSA_HAVE_OLED) && !defined(CONFIG_UPSA_REVA_OLED_VCC_FIRST)
#error "No OLED on plain Rev A except a VCC-first module: J3 powers a standard one backwards (use the reva-oled board target)"
#endif

#elif defined(CONFIG_UPSA_BOARD_REVA_OLED)

#define BOARD_NAME              "My UPS Pal Rev A (OLED)"
/* Same PCB and pin map as plain Rev A -- a second firmware IDENTITY, not a
 * second board. That is what makes it safe to hand to someone else: the OTA
 * client's own-board check keeps a plain Rev A board from ever being offered
 * a build that assumes a VCC-first display is fitted, and keeps a board
 * already running this build from being offered a plain Rev A release that
 * would leave its screen dark without saying why. */
#define BOARD_OTA_TARGET        "reva-oled"
#define BOARD_IMAGE_NAME        "ups-adaptor-reva-oled"
#define BOARD_PIN_VBUS_EN       GPIO_NUM_4   /* AP2171W EN, active high        */
#define BOARD_PIN_VBUS_FAULT    GPIO_NUM_5   /* AP2171W FLG, open drain, active low */
#define BOARD_PIN_LED_STATUS    GPIO_NUM_48  /* green, plain LED, 1k series    */
#define BOARD_PIN_LED_FAULT     GPIO_NUM_47  /* red, plain LED, 1k series      */
#define BOARD_PIN_I2C_SDA       GPIO_NUM_8
#define BOARD_PIN_I2C_SCL       GPIO_NUM_9
#define BOARD_PIN_FACTORY_BTN   GPIO_NUM_0   /* BOOT button, doubles as reset  */
#define BOARD_HAS_VBUS_SWITCH   1
#define BOARD_HAS_VBUS_FAULT    1
#define BOARD_HAS_STATUS_LEDS   1
/* This target exists FOR a VCC-first module: Kconfig ties the two together
 * (UPSA_BOARD_REVA_OLED selects UPSA_REVA_OLED_VCC_FIRST), so this catches an
 * edit that lets them drift apart rather than a module that was never fitted. */
#if !defined(CONFIG_UPSA_REVA_OLED_VCC_FIRST)
#error "UPSA_BOARD_REVA_OLED without UPSA_REVA_OLED_VCC_FIRST: Kconfig should have set both together"
#endif

#elif defined(CONFIG_UPSA_BOARD_REV_B)

#define BOARD_NAME              "My UPS Pal Rev B"
/* Picks the OTA asset: ups-adaptor-revb.bin. A Rev A board must never take a
 * Rev B image -- its N4 has a quarter of the flash the 16 MB partition table
 * describes -- and the image-name check below is what guarantees that. */
#define BOARD_OTA_TARGET        "revb"
#define BOARD_IMAGE_NAME        "ups-adaptor-revb"
/* Rev B keeps Rev A's pin map. The N16R8's octal PSRAM takes GPIO35-37, which
 * Rev A never used; D3/D4 moved on the board, not in the netlist. */
#define BOARD_PIN_VBUS_EN       GPIO_NUM_4   /* AP2171W EN, active high        */
#define BOARD_PIN_VBUS_FAULT    GPIO_NUM_5   /* AP2171W FLG, open drain, active low */
#define BOARD_PIN_LED_STATUS    GPIO_NUM_48  /* yellow, plain LED, 1k series   */
#define BOARD_PIN_LED_FAULT     GPIO_NUM_47  /* red, plain LED, 1k series      */
#define BOARD_PIN_I2C_SDA       GPIO_NUM_8
#define BOARD_PIN_I2C_SCL       GPIO_NUM_9
#define BOARD_PIN_FACTORY_BTN   GPIO_NUM_0   /* BOOT button, doubles as reset  */
#define BOARD_HAS_VBUS_SWITCH   1
#define BOARD_HAS_VBUS_FAULT    1
#define BOARD_HAS_STATUS_LEDS   1
/* The Rev B build is only right with sdkconfig.defaults.revb loaded. Without
 * it the image is laid out for 4 MB and no PSRAM: it would boot, and never see
 * the flash or memory Rev B exists for. */
#if !defined(CONFIG_ESPTOOLPY_FLASHSIZE_16MB) || !defined(CONFIG_SPIRAM_MODE_OCT)
#error "Rev B needs sdkconfig.defaults.revb: 16 MB flash and octal PSRAM (see that file)"
#endif

#elif defined(CONFIG_UPSA_BOARD_WAVESHARE_RS485)

/* The Genesis Controls UPS->BMS adapter. Same UPS reading as every board
 * here; the output is an isolated RS485 trunk to a building management
 * system, and most units never join a network at all. */
#define BOARD_NAME              "Genesis UPS-BMS adapter (Waveshare RS485-CAN)"
#define BOARD_OTA_TARGET        "genesis-rs485"
#define BOARD_IMAGE_NAME        "ups-genesis-rs485"

/* No load switch. The USB-C is a device port -- 5.1k CC pull-downs, VBUS fed
 * INTO the board through a Schottky -- so the UPS takes its 5 V from outside
 * and a wedged UPS is recovered by closing and re-enumerating it, the path a
 * self-powered UPS takes on Rev A anyway.
 *
 * Waveshare's own warning, worth repeating where someone will read it: do NOT
 * power the 7-36 V screw terminal and the USB-C at the same time. */
#define BOARD_PIN_VBUS_EN       GPIO_NUM_NC
#define BOARD_PIN_VBUS_FAULT    GPIO_NUM_NC

/* PWR, RS485 and CAN each have an LED, all wired to the supply or the
 * transceiver lines rather than to a GPIO. Good for watching trunk traffic at
 * the bench, useless as a status indicator, so the board reports none. */
#define BOARD_PIN_LED_STATUS    GPIO_NUM_NC
#define BOARD_PIN_LED_FAULT     GPIO_NUM_NC

/* The board carries a PCF85063 RTC and an SH1.0 battery header, but which
 * GPIOs its I2C runs on is NOT established -- Waveshare's demo does not touch
 * it and the wiki's pin table is an image. Left unconnected rather than
 * guessed; nothing uses I2C on this board today (no display). Worth settling
 * if we ever drive the RTC, which is the obvious way to make register 8 --
 * days since battery change -- work on a board with no network and so no
 * clock. */
#define BOARD_PIN_I2C_SDA       GPIO_NUM_NC
#define BOARD_PIN_I2C_SCL       GPIO_NUM_NC
#define BOARD_PIN_FACTORY_BTN   GPIO_NUM_0   /* BOOT, as on every S3 board */
#define BOARD_HAS_VBUS_SWITCH   0
#define BOARD_HAS_VBUS_FAULT    0
#define BOARD_HAS_STATUS_LEDS   0

/* RS485. From Waveshare's own demo (Arduino/examples/MAIN_WIFI_STA/WS_GPIO.h),
 * which is the only machine-readable source for this board's pin map.
 *
 * THE DRIVER ENABLE IS THE UART'S HARDWARE RTS, not a pin software toggles:
 * WS_RS485.cpp does setPins(-1,-1,-1,TXD1EN) and then
 * UART_MODE_RS485_HALF_DUPLEX. An output layer that keys DE itself is racing
 * the shift register -- it clips frames at the wire while everything on the
 * board still looks right, which is about the worst failure to diagnose on a
 * live trunk. Use the mode and let the peripheral do it. */
#define BOARD_HAS_RS485         1
#define BOARD_RS485_UART        1            /* UART_NUM_1 */
#define BOARD_PIN_RS485_TX      GPIO_NUM_17
#define BOARD_PIN_RS485_RX      GPIO_NUM_18
#define BOARD_PIN_RS485_DE      GPIO_NUM_21  /* hardware RTS, not a GPIO toggle */

/* CAN (TWAI) is fitted and deliberately unused: emulating Belimo's CAN
 * gas-detection protocol was rejected as proprietary and part of a UL 2075
 * listed system. Recorded so the pins are known to be spoken for. */
#define BOARD_PIN_CAN_TX        GPIO_NUM_15
#define BOARD_PIN_CAN_RX        GPIO_NUM_16

/* ESP32-S3R8 plus 16 MB external flash: Rev B's memory profile, so it loads
 * Rev B's settings. Without them the image is laid out for 4 MB and no PSRAM,
 * and the outage log, history and HTTPS this board is sized for are missing. */
#if !defined(CONFIG_ESPTOOLPY_FLASHSIZE_16MB) || !defined(CONFIG_SPIRAM_MODE_OCT)
#error "Waveshare RS485-CAN needs sdkconfig.defaults.revb first: 16 MB flash and octal PSRAM"
#endif

#elif defined(CONFIG_UPSA_BOARD_DEV_S3) || defined(CONFIG_UPSA_BOARD_DEV_S3_PSRAM)

/* A generic ESP32-S3 board, flashed by its owner. Nothing here is known about
 * it except what every S3 has: the native USB pins (GPIO19/20, owned by
 * usb_host) and the BOOT button on GPIO0. So no load switch, no FLAG line, no
 * LEDs and no display. Without the switch, recovery of a wedged UPS falls back
 * to closing and re-enumerating it (ups_hid's do_recover), which is the path a
 * self-powered UPS takes on Rev A anyway.
 *
 * Its own OTA target and image name, so it is never offered a Rev A or Rev B
 * image: none is published for it, and the image-name check refuses one if it
 * ever were. The check-in reports BOARD_IMAGE_NAME, which is how the project
 * counts these boards. */
#if defined(CONFIG_UPSA_BOARD_DEV_S3_PSRAM)
#define BOARD_NAME              "ESP32-S3 dev board (PSRAM)"
#define BOARD_OTA_TARGET        "dev-s3-psram"
#define BOARD_IMAGE_NAME        "ups-adaptor-dev-s3-psram"
/* Rev B's module (N16R8) and memory settings: sdkconfig.defaults.revb then
 * sdkconfig.defaults.dev-s3-psram. Without them the image is laid out for 4 MB
 * and no PSRAM, and the Rev B features it was built for are missing. */
#if !defined(CONFIG_ESPTOOLPY_FLASHSIZE_16MB) || !defined(CONFIG_SPIRAM_MODE_OCT)
#error "The PSRAM dev board needs sdkconfig.defaults.revb then sdkconfig.defaults.dev-s3-psram: 16 MB flash and octal PSRAM"
#endif
#else
#define BOARD_NAME              "ESP32-S3 dev board"
#define BOARD_OTA_TARGET        "dev-s3"
#define BOARD_IMAGE_NAME        "ups-adaptor-dev-s3"
#endif
#define BOARD_PIN_VBUS_EN       GPIO_NUM_NC  /* no load switch: VBUS is wired */
#define BOARD_PIN_VBUS_FAULT    GPIO_NUM_NC
#define BOARD_PIN_LED_STATUS    GPIO_NUM_NC  /* many have an RGB LED; not driven */
#define BOARD_PIN_LED_FAULT     GPIO_NUM_NC
#define BOARD_PIN_I2C_SDA       GPIO_NUM_8   /* unused: no display on this target */
#define BOARD_PIN_I2C_SCL       GPIO_NUM_9
#define BOARD_PIN_FACTORY_BTN   GPIO_NUM_0   /* BOOT button, as on every S3 board */
#define BOARD_HAS_VBUS_SWITCH   0
#define BOARD_HAS_VBUS_FAULT    0
#define BOARD_HAS_STATUS_LEDS   0

#else
#error "No target board selected. Run idf.py menuconfig -> My UPS Pal board."
#endif

#if defined(CONFIG_UPSA_REVA_OLED_VCC_FIRST)
/* J3 pin 3 is the board's SDA net (GPIO8) and pin 4 its SCL (GPIO9); a
 * VCC-first module puts SCL on pin 3 and SDA on pin 4. The board's nets are
 * unchanged -- BOARD_PIN_I2C_* still describe them -- only the display swaps. */
#define BOARD_OLED_SDA          BOARD_PIN_I2C_SCL
#define BOARD_OLED_SCL          BOARD_PIN_I2C_SDA
#endif

/* Boards with no RS485 transceiver, which is every one but the Waveshare
 * adapter. Application code tests BOARD_HAS_RS485, never the board choice. */
#ifndef BOARD_HAS_RS485
#define BOARD_HAS_RS485 0
#endif

/* The pins the DISPLAY uses for SDA and SCL. The board's own I2C nets, unless
 * a board block above says the module in its socket crosses them. */
#ifndef BOARD_OLED_SDA
#define BOARD_OLED_SDA          BOARD_PIN_I2C_SDA
#define BOARD_OLED_SCL          BOARD_PIN_I2C_SCL
#endif

/* The MAC that names this board in the fleet heartbeat and on the status page:
 * the Wi-Fi station's where there is a radio, the Ethernet MAC where there is
 * not (a P4 has no Wi-Fi MAC to read). */
#ifdef CONFIG_UPSA_NET_ETHERNET
#define BOARD_ID_MAC_TYPE       ESP_MAC_ETH
#else
#define BOARD_ID_MAC_TYPE       ESP_MAC_WIFI_STA
#endif

/* Bring up board-owned GPIO. Call once, first, from app_main. */
esp_err_t board_init(void);

/* Assert or deassert VBUS on the USB-A host port.
 * Returns ESP_ERR_NOT_SUPPORTED on boards without a load switch, so callers
 * can log-and-continue rather than treating it as fatal. */
esp_err_t board_vbus_set(bool on);

/* Drop VBUS for CONFIG_UPSA_VBUS_CYCLE_MS, then restore it. Blocks.
 * This is the recovery path for a wedged UPS USB interface. */
esp_err_t board_vbus_cycle(void);

/* True if the load switch is reporting an overcurrent or thermal fault.
 * Always false where the board has no FLAG line. */
bool board_vbus_fault(void);

/* True while the factory-reset button is held (active low). */
bool board_factory_button_pressed(void);

/* Coarse device state, shown on two plain LEDs.
 *
 * Semantic rather than per-pin: Rev A has a green and a red LED, a later board
 * may not, and callers should not know or care. Deliberately coarse -- the OLED
 * and web UI carry anything that needs detail.
 *
 * THE OLED MUST SHOW EVERY STATE HERE. On Rev A the display module sits
 * directly over both LEDs, so with it fitted the LEDs cannot be seen and the
 * screen is the only local indicator. display.c renders board_status_get()
 * through a switch with no default, so a state added here and not there is a
 * -Wswitch error, and tools/test/test_display_parity.py checks the same thing
 * without a build. */
typedef enum {
    BOARD_STATUS_BOOTING,       /* green solid                    */
    BOARD_STATUS_PROVISIONING,  /* green fast blink: setup AP up  */
    BOARD_STATUS_CONNECTING,    /* green slow blink               */
    BOARD_STATUS_ONLINE,        /* green solid, red off           */
    BOARD_STATUS_NO_UPS,        /* green solid, red slow blink    */
    BOARD_STATUS_FAULT,         /* red solid: VBUS fault          */
} board_status_t;

void board_status_set(board_status_t state);

/* The state last set, for the OLED to mirror what the LEDs are showing. */
board_status_t board_status_get(void);

#endif /* UPSA_BOARD_H */
