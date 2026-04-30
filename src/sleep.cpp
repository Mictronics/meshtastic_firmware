#include "configuration.h"

#include "Default.h"
#include "MeshRadio.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "PowerMon.h"
#include "detect/LoRaRadioType.h"
#include "error.h"
#include "main.h"
#include "sleep.h"
#include "target_specific.h"

#ifdef ARCH_ESP32
// "esp_pm_config_esp32_t is deprecated, please include esp_pm.h and use esp_pm_config_t instead"
#include "esp32/pm.h"
#include "esp_pm.h"
#if HAS_WIFI
#include "mesh/wifi/WiFiAPClient.h"
#endif
#include "rom/rtc.h"
#include <RadioLib.h>
#include <driver/rtc_io.h>
#include <driver/uart.h>

esp_sleep_source_t wakeCause; // the reason we booted this time
#endif
#include "Throttle.h"

#ifndef INCLUDE_vTaskSuspend
#define INCLUDE_vTaskSuspend 0
#endif

/// Called to tell observers we are rebooting ASAP.  Must return 0
Observable<void *> notifyReboot;

// deep sleep support
RTC_DATA_ATTR int bootCount = 0;

/**
 * Control CPU core speed (80MHz vs 240MHz)
 *
 * We leave CPU at full speed during init, but once loop is called switch to low speed (for a 50% power savings)
 *
 */
void setCPUFast(bool on)
{
#if defined(ARCH_ESP32) && HAS_WIFI && !HAS_TFT

    if (isWifiAvailable()) {
        /*
         *
         * There's a newly introduced bug in the espressif framework where WiFi is
         *   unstable when the frequency is less than 240MHz.
         *
         *   This mostly impacts WiFi AP mode but we'll bump the frequency for
         *     all WiFi use cases.
         * (Added: Dec 23, 2021 by Jm Casler)
         */
#ifndef CONFIG_IDF_TARGET_ESP32C3
        LOG_DEBUG("Set CPU to 240MHz because WiFi is in use");
        setCpuFrequencyMhz(240);
#endif
        return;
    }

// The Heltec LORA32 V1 runs at 26 MHz base frequency and doesn't react well to switching to 80 MHz...
#if !defined(ARDUINO_HELTEC_WIFI_LORA_32) && !defined(CONFIG_IDF_TARGET_ESP32C3)
    setCpuFrequencyMhz(on ? 240 : 80);
#endif

#endif
}
