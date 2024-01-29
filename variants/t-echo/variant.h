/*
 Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
 Copyright (c) 2016 Sandeep Mistry All right reserved.
 Copyright (c) 2018, Adafruit Industries (adafruit.com)

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU Lesser General Public License for more details.
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_TTGO_EINK_V1_0_
#define _VARIANT_TTGO_EINK_V1_0_

/** Master clock frequency */
#define VARIANT_MCK (64000000ul)

#define USE_LFXO // Board uses 32khz crystal for LF

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define TTGO_T_ECHO

// Build specific code for T-Echo based remote router.
#define T_ECHO_ROUTER

#ifdef T_ECHO_ROUTER
// There will be no screen on the router
#undef HAS_SCREEN
// Disable shutdown functionality
#undef HAS_CPU_SHUTDOWN
#define HAS_CPU_SHUTDOWN 0
/* Hardcoded node configuration */
// Primary channel
#define PRIMARY_CH_NAME "Private"
#define PRIMARY_PSK                                                                                                              \
    {                                                                                                                            \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  \
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00                                               \
    }
// Admin channel
// Name set to "admin"
#define ADMIN_PSK                                                                                                                \
    {                                                                                                                            \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  \
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00                                               \
    }
#error Change PSK!

// Public Germany wide channel
#define CHANNEL3_NAME "Deutsch"
#define CHANNEL3_PSK                                                                                                             \
    {                                                                                                                            \
        0x2b, 0xbb, 0x63, 0x7b, 0xa5, 0x52, 0x0d, 0xd5, 0x5e, 0x36, 0x10, 0x0f, 0xd6, 0x33, 0xf7, 0xc7, 0x77, 0x6f, 0x34, 0x5b,  \
            0x56, 0x3b, 0x80, 0x9e, 0xde, 0x73, 0xec, 0xa2, 0x85, 0x42, 0x0c, 0xa8,                                              \
    }
#endif

// Number of pins defined in PinDescription array
#define PINS_COUNT (48)
#define NUM_DIGITAL_PINS (48)
#define NUM_ANALOG_INPUTS (1)
#define NUM_ANALOG_OUTPUTS (0)

// LEDs
#ifdef T_ECHO_ROUTER
#define PIN_LED1 (0 + 8) // LED disabled, output on unused pin
#else
#define PIN_LED1 (0 + 14) // blue (confirmed on board marked v1.0, date 2021-6-28)
#endif
#define PIN_LED2 (32 + 1) // green
#define PIN_LED3 (32 + 3) // red

#define LED_RED PIN_LED3
#define LED_BLUE PIN_LED1
#define LED_GREEN PIN_LED2

#define LED_BUILTIN LED_BLUE
#define LED_CONN PIN_GREEN

#define LED_STATE_ON 0 // State when LED is lit
#define LED_INVERTED 1

/*
 * Buttons
 */
#define PIN_BUTTON1 (32 + 10)
#define PIN_BUTTON2 (0 + 18)      // 0.18 is labeled on the board as RESET but we configure it in the bootloader as a regular GPIO
#define PIN_BUTTON_TOUCH (0 + 11) // 0.11 is the soft touch button on T-Echo

/*
 * Remove touch button functionality.
 * Pin 0.11 used for intrusion detection.
 * Requires hardware change:
 * - remove TTP223 and bridge its pin 1 to 3.
 * - connect intrusion switch between touch sensor pad and GND.
 * In configuration:
 * - enable detection sensor module
 * - set 'Monitor Pin' to 11
 * - set 'Detection Triggered High' false/low
 * - set 'Use Pullup' enabled
 */
#undef PIN_BUTTON_TOUCH
#ifdef BUTTON_PIN_TOUCH
#undef BUTTON_PIN_TOUCH
#endif

/*
 * At intrusion detection event send last known position on primary channel.
 */
#define INTRUSION_DETECTION_POSITION

/*
 * Analog pins
 */
#define PIN_A0 (4) // Battery ADC

#define BATTERY_PIN PIN_A0

static const uint8_t A0 = PIN_A0;

#define ADC_RESOLUTION 14

#define PIN_NFC1 (9)
#define PIN_NFC2 (10)

/*
 * Serial interfaces
 */

/*
No longer populated on PCB
*/
// #define PIN_SERIAL2_RX (0 + 6)
// #define PIN_SERIAL2_TX (0 + 8)
//  #define PIN_SERIAL2_EN (0 + 17)

/**
    Wire Interfaces
    */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA (26)
#define PIN_WIRE_SCL (27)

/* touch sensor, active high */

#define TP_SER_IO (0 + 11)

#define PIN_RTC_INT (0 + 16) // Interrupt from the PCF8563 RTC

/*
External serial flash WP25R1635FZUIL0
*/

// QSPI Pins
#define PIN_QSPI_SCK (32 + 14)
#define PIN_QSPI_CS (32 + 15)
#define PIN_QSPI_IO0 (32 + 12) // MOSI if using two bit interface
#define PIN_QSPI_IO1 (32 + 13) // MISO if using two bit interface
#define PIN_QSPI_IO2 (0 + 7)   // WP if using two bit interface (i.e. not used)
#define PIN_QSPI_IO3 (0 + 5)   // HOLD if using two bit interface (i.e. not used)

// On-board QSPI Flash
#define EXTERNAL_FLASH_DEVICES MX25R1635F
#define EXTERNAL_FLASH_USE_QSPI

/*
 * Lora radio
 */

#define USE_SX1262
#define USE_SX1268
#define SX126X_CS (0 + 24) // FIXME - we really should define LORA_CS instead
#define SX126X_DIO1 (0 + 20)
// Note DIO2 is attached internally to the module to an analog switch for TX/RX switching
#define SX1262_DIO3                                                                                                              \
    (0 + 21) // This is used as an *output* from the sx1262 and connected internally to power the tcxo, do not drive from the main
             // CPU?
#define SX126X_BUSY (0 + 17)
#define SX126X_RESET (0 + 25)
// Not really an E22 but TTGO seems to be trying to clone that
#define SX126X_DIO2_AS_RF_SWITCH
#define SX126X_DIO3_TCXO_VOLTAGE 1.8
// Internally the TTGO module hooks the SX1262-DIO2 in to control the TX/RX switch (which is the default for the sx1262interface
// code)

// #define LORA_DISABLE_SENDING // Define this to disable transmission for testing (power testing etc...)

// #undef SX126X_CS

/*
 * eink display pins
 */

#define PIN_EINK_EN (32 + 11) // Note: this is really just backlight power
#define PIN_EINK_CS (0 + 30)
#define PIN_EINK_BUSY (0 + 3)
#define PIN_EINK_DC (0 + 28)
#define PIN_EINK_RES (0 + 2)
#define PIN_EINK_SCLK (0 + 31)
#define PIN_EINK_MOSI (0 + 29) // also called SDI

// Controls power for the eink display - Board power is enabled either by VBUS from USB or the CPU asserting PWR_ON
// FIXME - I think this is actually just the board power enable - it enables power to the CPU also
#define PIN_EINK_PWR_ON (0 + 12)

#define USE_EINK

#define PIN_SPI1_MISO                                                                                                            \
    (32 + 7) // FIXME not really needed, but for now the SPI code requires something to be defined, pick an used GPIO
#define PIN_SPI1_MOSI PIN_EINK_MOSI
#define PIN_SPI1_SCK PIN_EINK_SCLK

/*
 * GPS pins
 */

#define GPS_L76K
#define PIN_GPS_REINIT (32 + 5) // An output to reset L76K GPS. As per datasheet, low for > 100ms will reset the L76K

#define PIN_GPS_STANDBY (32 + 2) // An output to wake GPS, low means allow sleep, high means force wake
// Seems to be missing on this new board
// #define PIN_GPS_PPS (32 + 4)  // Pulse per second input from the GPS
#define PIN_GPS_TX (32 + 9) // This is for bits going TOWARDS the CPU
#define PIN_GPS_RX (32 + 8) // This is for bits going TOWARDS the GPS

#define GPS_THREAD_INTERVAL 50

#define PIN_SERIAL1_RX PIN_GPS_TX
#define PIN_SERIAL1_TX PIN_GPS_RX

// PCF8563 RTC Module
#define PCF8563_RTC 0x51

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 2

// For LORA, spi 0
#define PIN_SPI_MISO (0 + 23)
#define PIN_SPI_MOSI (0 + 22)
#define PIN_SPI_SCK (0 + 19)

#define PIN_PWR_EN (0 + 6)

// To debug via the segger JLINK console rather than the CDC-ACM serial device
// #define USE_SEGGER

// Battery
// The battery sense is hooked to pin A0 (4)
// it is defined in the anlaolgue pin section of this file
// and has 12 bit resolution
#define BATTERY_SENSE_RESOLUTION_BITS 12
#define BATTERY_SENSE_RESOLUTION 4096.0
// Definition of milliVolt per LSB => 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_MV_PER_LSB (0.73242188F)
// Voltage divider value => 100K + 100K voltage divider on VBAT = (100K / (100K + 100K))
#define VBAT_DIVIDER (0.5F)
// Compensation factor for the VBAT divider
#define VBAT_DIVIDER_COMP (2.0F)
// Fixed calculation of milliVolt from compensation value
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)
#undef AREF_VOLTAGE
#define AREF_VOLTAGE 3.0
#define VBAT_AR_INTERNAL AR_INTERNAL_3_0
#define ADC_MULTIPLIER VBAT_DIVIDER_COMP
#define VBAT_RAW_TO_SCALED(x) (REAL_VBAT_MV_PER_LSB * x)

#define HAS_RTC 1

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif