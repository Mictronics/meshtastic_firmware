#include "MeshRadio.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "PowerFSM.h"
#include "PowerMon.h"
#include "RadioLibInterface.h"
#include "ReliableRouter.h"
#include "airtime.h"
#include "configuration.h"
#include "power/PowerHAL.h"

#include "FSCommon.h"
#include "RTC.h"
#include "SPILock.h"
#include "Throttle.h"
#include "concurrency/OSThread.h"
#include "concurrency/Periodic.h"
#include "detect/ScanI2C.h"
#include "error.h"
#include "power.h"

#if !MESHTASTIC_EXCLUDE_I2C
#include "detect/ScanI2CConsumer.h"
#include "detect/ScanI2CTwoWire.h"
#include <Wire.h>
#endif
#include "main.h"
#include "mesh/generated/meshtastic/config.pb.h"
#include "meshUtils.h"
#include "modules/Modules.h"
#include "sleep.h"
#include "target_specific.h"
#include <memory>
#include <utility>

#ifdef ARCH_ESP32
#include "freertosinc.h"
#if !MESHTASTIC_EXCLUDE_WEBSERVER
#include "mesh/http/WebServer.h"
#endif
#if !MESHTASTIC_EXCLUDE_BLUETOOTH
#include "nimble/NimbleBluetooth.h"
NimbleBluetooth *nimbleBluetooth = nullptr;
#endif
#endif

#ifdef ARCH_NRF52
#include "NRF52Bluetooth.h"
NRF52Bluetooth *nrf52Bluetooth = nullptr;
#endif

#if HAS_WIFI || defined(USE_WS5500)
#include "mesh/api/WiFiServerAPI.h"
#include "mesh/wifi/WiFiAPClient.h"
#endif

#if HAS_ETHERNET && !defined(USE_WS5500)
#include "mesh/api/ethServerAPI.h"
#include "mesh/eth/ethClient.h"
#endif

#include "LLCC68Interface.h"
#include "LR1110Interface.h"
#include "LR1120Interface.h"
#include "LR1121Interface.h"
#include "RF95Interface.h"
#include "SX1262Interface.h"
#include "SX1268Interface.h"
#include "SX1280Interface.h"
#include "detect/LoRaRadioType.h"

#ifdef ARCH_STM32WL
#include "STM32WLE5JCInterface.h"
#endif

#if defined(ARCH_PORTDUINO)
#include "platform/portduino/SimRadio.h"
#endif

#ifdef ARCH_PORTDUINO
#include "linux/LinuxHardwareI2C.h"
#include "mesh/raspihttp/PiWebServer.h"
#include "platform/portduino/PortduinoGlue.h"
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#endif

#include "PowerFSMThread.h"

#ifdef HAS_I2S
#include "AudioThread.h"
AudioThread *audioThread = nullptr;
#endif

#ifdef HAS_UDP_MULTICAST
#include "mesh/udp/UdpMulticastHandler.h"
UdpMulticastHandler *udpHandler = nullptr;
#endif

#if defined(TCXO_OPTIONAL)
float tcxoVoltage = SX126X_DIO3_TCXO_VOLTAGE; // if TCXO is optional, put this here so it can be changed further down.
#endif

#if defined(HW_SPI1_DEVICE) && defined(ARCH_ESP32)
SPIClass SPI1(HSPI);
#endif

using namespace concurrency;

volatile static const char slipstreamTZString[] = {USERPREFS_TZ_STRING};

// Global power status
meshtastic::PowerStatus *powerStatus = new meshtastic::PowerStatus();

// Global Node status
meshtastic::NodeStatus *nodeStatus = new meshtastic::NodeStatus();

// Global Bluetooth status
meshtastic::BluetoothStatus *bluetoothStatus = new meshtastic::BluetoothStatus();

// Scan for I2C Devices

/// The I2C address of our display (if found)
ScanI2C::DeviceAddress screen_found = ScanI2C::ADDRESS_NONE;

// The I2C address of the cardkb or RAK14004 (if found)
ScanI2C::DeviceAddress cardkb_found = ScanI2C::ADDRESS_NONE;
// 0x02 for RAK14004, 0x00 for cardkb, 0x10 for T-Deck
uint8_t kb_model;
// global bool to record that a kb is present
bool kb_found = false;
// global bool to record that on-screen keyboard (OSK) is present
bool osk_found = false;

// The I2C address of the RTC Module (if found)
ScanI2C::DeviceAddress rtc_found = ScanI2C::ADDRESS_NONE;
// The I2C address of the RGB LED (if found)
ScanI2C::FoundDevice rgb_found = ScanI2C::FoundDevice(ScanI2C::DeviceType::NONE, ScanI2C::ADDRESS_NONE);
/// The I2C address of our Air Quality Indicator (if found)
ScanI2C::DeviceAddress aqi_found = ScanI2C::ADDRESS_NONE;

#ifdef HAS_DRV2605
Adafruit_DRV2605 drv;
#endif

bool isVibrating = false;

bool eink_found = true;

// Holds the timestamp we got during RX
// Update in RadioLibInterface::handleReceiveInterrupt()
// Used for RX watchdog to detect dead radio chip.
uint32_t lastRxMsec = 0;

bool pauseBluetoothLogging = false;

bool pmu_found;

#if !MESHTASTIC_EXCLUDE_I2C
// Array map of sensor types with i2c address and wire as we'll find in the i2c scan
std::pair<uint8_t, TwoWire *> nodeTelemetrySensorsMap[_meshtastic_TelemetrySensorType_MAX + 1] = {};
#endif

Router *router = NULL; // Users of router don't care what sort of subclass implements that API

const char *firmware_version = optstr(APP_VERSION_SHORT);

const char *getDeviceName()
{
    uint8_t dmac[6];

    getMacAddr(dmac);

    // Meshtastic_ab3c or Shortname_abcd
    static char name[20];
    snprintf(name, sizeof(name), "%02x%02x", dmac[4], dmac[5]);
    // if the shortname exists and is NOT the new default of ab3c, use it for BLE name.
    if (strcmp(owner.short_name, name) != 0) {
        snprintf(name, sizeof(name), "%s_%02x%02x", owner.short_name, dmac[4], dmac[5]);
    } else {
        snprintf(name, sizeof(name), "Meshtastic_%02x%02x", dmac[4], dmac[5]);
    }
    return name;
}

uint32_t timeLastPowered = 0;

static OSThread *powerFSMthread;

RadioInterface *rIf = NULL;
RadioLibHal *RadioLibHAL = NULL;

/**
 * Some platforms (nrf52) might provide an alterate version that suppresses calling delay from sleep.
 */
__attribute__((weak, noinline)) bool loopCanSleep()
{
    return true;
}

// Weak empty variant initialization function.
// May be redefined by variant files.
void lateInitVariant() __attribute__((weak));
void lateInitVariant() {}

void earlyInitVariant() __attribute__((weak));
void earlyInitVariant() {}

// NRF52 (and probably other platforms) can report when system is in power failure mode
// (eg. too low battery voltage) and operating it is unsafe (data corruption, bootloops, etc).
// For example NRF52 will prevent any flash writes in that case automatically
// (but it causes issues we need to handle).
// This detection is independent from whatever ADC or dividers used in Meshtastic
// boards and is internal to chip.

// we use powerHAL layer to get this info and delay booting until power level is safe

// wait until power level is safe to continue booting (to avoid bootloops)
// blink user led in 3 flashes sequence to indicate what is happening
void waitUntilPowerLevelSafe()
{
    while (powerHAL_isPowerLevelSafe() == false) {
        // sleep for 2s
        delay(2000);
    }
}

/**
 * Print info as a structured log message (for automated log processing)
 */
void printInfo()
{
    LOG_INFO("S:B:%d,%s,%s,%s", HW_VENDOR, optstr(APP_VERSION), optstr(APP_ENV), optstr(APP_REPO));
}
#ifndef PIO_UNIT_TESTING
void setup()
{

    // initialize power HAL layer as early as possible
    powerHAL_init();

#ifdef LED_POWER
    pinMode(LED_POWER, OUTPUT);
    digitalWrite(LED_POWER, LED_STATE_ON);
#endif

    // prevent booting if device is in power failure mode
    // boot sequence will follow when battery level raises to safe mode
    waitUntilPowerLevelSafe();

    // Defined in variant.cpp for early init code
    earlyInitVariant();

#if defined(PIN_POWER_EN)
    pinMode(PIN_POWER_EN, OUTPUT);
    digitalWrite(PIN_POWER_EN, HIGH);
#endif

#ifdef LED_NOTIFICATION
    pinMode(LED_NOTIFICATION, OUTPUT);
    digitalWrite(LED_NOTIFICATION, HIGH ^ LED_STATE_ON);
#endif

#ifdef WIFI_LED
    pinMode(WIFI_LED, OUTPUT);
    digitalWrite(WIFI_LED, LOW);
#endif

#ifdef BLE_LED
    pinMode(BLE_LED, OUTPUT);
    digitalWrite(BLE_LED, LED_STATE_OFF);
#endif

    concurrency::hasBeenSetup = true;
#if ARCH_PORTDUINO
    SPISettings spiSettings(portduino_config.spiSpeed, MSBFIRST, SPI_MODE0);
#else
    SPISettings spiSettings(4000000, MSBFIRST, SPI_MODE0);
#endif

#ifdef USE_SEGGER
    auto mode = false ? SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL : SEGGER_RTT_MODE_NO_BLOCK_TRIM;
#ifdef NRF52840_XXAA
    auto buflen = 4096; // this board has a fair amount of ram
#else
    auto buflen = 256; // this board has a fair amount of ram
#endif
    SEGGER_RTT_ConfigUpBuffer(SEGGER_STDOUT_CH, NULL, NULL, buflen, mode);
#endif

#ifdef DEBUG_PORT
    consoleInit(); // Set serial baud rate and init our mesh console
#endif

#ifdef UNPHONE
    unphone.printStore();
#endif

#if ARCH_PORTDUINO
    RTCQuality ourQuality = RTCQualityDevice;

    std::string timeCommandResult = exec("timedatectl status | grep synchronized | grep yes -c");
    if (timeCommandResult[0] == '1') {
        ourQuality = RTCQualityNTP;
    }

    struct timeval tv;
    tv.tv_sec = time(NULL);
    tv.tv_usec = 0;
    perhapsSetRTC(ourQuality, &tv);
#endif

    powerMonInit();

    LOG_INFO("\n\n//\\ E S H T /\\ S T / C\n");

#if defined(ARCH_ESP32) && defined(BOARD_HAS_PSRAM)
#ifndef SENSECAP_INDICATOR
    // use PSRAM for malloc calls > 256 bytes
    heap_caps_malloc_extmem_enable(256);
#endif
#endif

#if defined(DEBUG_MUTE) && defined(DEBUG_PORT)
    DEBUG_PORT.printf("\r\n\r\n//\\ E S H T /\\ S T / C\r\n");
    DEBUG_PORT.printf("Version %s for %s from %s\r\n", optstr(APP_VERSION), optstr(APP_ENV), optstr(APP_REPO));
    DEBUG_PORT.printf("Debug mute is enabled, there will be no serial output.\r\n");
#endif

    initDeepSleep();

#if defined(MODEM_POWER_EN)
    pinMode(MODEM_POWER_EN, OUTPUT);
    digitalWrite(MODEM_POWER_EN, LOW);
#endif

#if defined(MODEM_PWRKEY)
    pinMode(MODEM_PWRKEY, OUTPUT);
    digitalWrite(MODEM_PWRKEY, LOW);
#endif

#if defined(LORA_TCXO_GPIO)
    pinMode(LORA_TCXO_GPIO, OUTPUT);
    digitalWrite(LORA_TCXO_GPIO, HIGH);
#endif

#if defined(VEXT_ENABLE)
    pinMode(VEXT_ENABLE, OUTPUT);
    digitalWrite(VEXT_ENABLE, VEXT_ON_VALUE); // turn on the display power
#endif

#if defined(BIAS_T_ENABLE)
    pinMode(BIAS_T_ENABLE, OUTPUT);
    digitalWrite(BIAS_T_ENABLE, BIAS_T_VALUE); // turn on 5V for GPS Antenna
#endif

#if defined(VTFT_CTRL)
    pinMode(VTFT_CTRL, OUTPUT);
    digitalWrite(VTFT_CTRL, LOW);
#endif

#ifdef RESET_OLED
    pinMode(RESET_OLED, OUTPUT);
    digitalWrite(RESET_OLED, 1);
    delay(2);
    digitalWrite(RESET_OLED, 0);
    delay(10);
    digitalWrite(RESET_OLED, 1);
#endif

#ifdef SENSOR_POWER_CTRL_PIN
    pinMode(SENSOR_POWER_CTRL_PIN, OUTPUT);
    digitalWrite(SENSOR_POWER_CTRL_PIN, SENSOR_POWER_ON);
#endif

#ifdef SENSOR_GPS_CONFLICT
    bool sensor_detected = false;
#endif
#ifdef PERIPHERAL_WARMUP_MS
    // Some peripherals may require additional time to stabilize after power is connected
    // e.g. I2C on Heltec Vision Master
    LOG_INFO("Wait for peripherals to stabilize");
    delay(PERIPHERAL_WARMUP_MS);
#endif
    initSPI();

    OSThread::setup();

    fsInit();

#if !MESHTASTIC_EXCLUDE_I2C
#if defined(I2C_SDA1) && defined(ARCH_RP2040)
    Wire1.setSDA(I2C_SDA1);
    Wire1.setSCL(I2C_SCL1);
    Wire1.begin();
#elif defined(I2C_SDA1) && !defined(ARCH_RP2040)
    Wire1.begin(I2C_SDA1, I2C_SCL1);
#elif WIRE_INTERFACES_COUNT == 2
    Wire1.begin();
#endif

#if defined(I2C_SDA) && defined(ARCH_RP2040)
    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
    Wire.begin();
#elif defined(I2C_SDA) && !defined(ARCH_RP2040)
    LOG_INFO("Starting Bus with (SDA) %d and (SCL) %d: ", I2C_SDA, I2C_SCL);
    Wire.begin(I2C_SDA, I2C_SCL);
#elif defined(ARCH_PORTDUINO)
    if (portduino_config.i2cdev != "") {
        LOG_INFO("Use %s as I2C device", portduino_config.i2cdev.c_str());
        Wire.begin(portduino_config.i2cdev.c_str());
    } else {
        LOG_INFO("No I2C device configured, Skip");
    }
#elif HAS_WIRE
    Wire.begin();
#endif
#endif

#if defined(M5STACK_UNITC6L)
    pinMode(LORA_CS, OUTPUT);
    digitalWrite(LORA_CS, 1);
    c6l_init();
#endif

#ifdef PIN_LCD_RESET
    // FIXME - move this someplace better, LCD is at address 0x3F
    pinMode(PIN_LCD_RESET, OUTPUT);
    digitalWrite(PIN_LCD_RESET, 0);
    delay(1);
    digitalWrite(PIN_LCD_RESET, 1);
    delay(1);
#endif

#ifdef AQ_SET_PIN
    // RAK-12039 set pin for Air quality sensor. Detectable on I2C after ~3 seconds, so we need to rescan later
    pinMode(AQ_SET_PIN, OUTPUT);
    digitalWrite(AQ_SET_PIN, HIGH);
#endif

    // Currently only the tbeam has a PMU
    // PMU initialization needs to be placed before i2c scanning
    power = new Power();
    power->setStatusHandler(powerStatus);
    powerStatus->observe(&power->newStatus);
    power->setup(); // Must be after status handler is installed, so that handler gets notified of the initial configuration

#if !MESHTASTIC_EXCLUDE_I2C
    // We need to scan here to decide if we have a screen for nodeDB.init() and because power has been applied to
    // accessories
    auto i2cScanner = std::unique_ptr<ScanI2CTwoWire>(new ScanI2CTwoWire());
#if HAS_WIRE
    LOG_INFO("Scan for i2c devices");
#endif

#if defined(I2C_SDA1) || (defined(NRF52840_XXAA) && (WIRE_INTERFACES_COUNT == 2))
    i2cScanner->scanPort(ScanI2C::I2CPort::WIRE1);
#endif

#if defined(I2C_SDA)
    i2cScanner->scanPort(ScanI2C::I2CPort::WIRE);
#elif defined(ARCH_PORTDUINO)
    if (portduino_config.i2cdev != "") {
        LOG_INFO("Scan for i2c devices");
        i2cScanner->scanPort(ScanI2C::I2CPort::WIRE);
    }
#elif HAS_WIRE
    i2cScanner->scanPort(ScanI2C::I2CPort::WIRE);
#endif

    auto i2cCount = i2cScanner->countDevices();
    if (i2cCount == 0) {
        LOG_INFO("No I2C devices found");
    } else {
        LOG_INFO("%i I2C devices found", i2cCount);
#ifdef SENSOR_GPS_CONFLICT
        sensor_detected = true;
#endif
    }

    auto rtc_info = i2cScanner->firstRTC();
    rtc_found = rtc_info.type != ScanI2C::DeviceType::NONE ? rtc_info.address : rtc_found;
    /*
     * There are a bunch of sensors that have no further logic than to be found and stuffed into the
     * nodeTelemetrySensorsMap singleton. This wraps that logic in a temporary scope to declare the temporary field
     * "found".
     */
    scannerToSensorsMap(i2cScanner, ScanI2C::DeviceType::INA260, meshtastic_TelemetrySensorType_INA260);
    scannerToSensorsMap(i2cScanner, ScanI2C::DeviceType::INA226, meshtastic_TelemetrySensorType_INA226);
    scannerToSensorsMap(i2cScanner, ScanI2C::DeviceType::INA219, meshtastic_TelemetrySensorType_INA219);
    scannerToSensorsMap(i2cScanner, ScanI2C::DeviceType::INA3221, meshtastic_TelemetrySensorType_INA3221);
#endif

#ifdef HAS_SDCARD
    setupSDCard();
#endif

    // Hello
    printInfo();
#ifdef BUILD_EPOCH
    LOG_INFO("Build timestamp: %ld", BUILD_EPOCH);
#endif

#ifdef ARCH_ESP32
    esp32Setup();
#endif

#ifdef ARCH_NRF52
    nrf52Setup();
#endif

#ifdef ARCH_RP2040
    rp2040Setup();
#endif

#if HAS_EXT_WATCHDOG && defined(EXT_WATCHDOG_TRIGGER)
    // Setup GPIO as external watchdog trigger output
    pinMode(EXT_WATCHDOG_TRIGGER, OUTPUT);
    digitalWrite(EXT_WATCHDOG_TRIGGER, LOW);
#endif

    // We do this as early as possible because this loads preferences from flash
    // but we need to do this after main cpu init (esp32setup), because we need the random seed set
    nodeDB = new NodeDB;

    // If we're taking on the repeater role, use NextHopRouter and turn off 3V3_S rail because peripherals are not needed
    if (config.device.role == meshtastic_Config_DeviceConfig_Role_REPEATER) {
        router = new NextHopRouter();
#ifdef PIN_3V3_EN
        digitalWrite(PIN_3V3_EN, LOW);
#endif
    } else
        router = new ReliableRouter();

    // only play start melody when role is not tracker or sensor
    if (config.power.is_power_saving == true &&
        IS_ONE_OF(config.device.role, meshtastic_Config_DeviceConfig_Role_TRACKER,
                  meshtastic_Config_DeviceConfig_Role_TAK_TRACKER, meshtastic_Config_DeviceConfig_Role_SENSOR))
        LOG_DEBUG("Tracker/Sensor: Skip start melody");

#ifdef HAS_DRV2605
#if defined(PIN_DRV_EN)
    pinMode(PIN_DRV_EN, OUTPUT);
    digitalWrite(PIN_DRV_EN, HIGH);
    delay(10);
#endif
    drv.begin();
    drv.selectLibrary(1);
    // I2C trigger by sending 'go' command
    drv.setMode(DRV2605_MODE_INTTRIG);
#endif

    // Init our SPI controller (must be before screen and lora)
#ifdef ARCH_RP2040
#ifdef HW_SPI1_DEVICE
    SPI1.setSCK(LORA_SCK);
    SPI1.setTX(LORA_MOSI);
    SPI1.setRX(LORA_MISO);
    pinMode(LORA_CS, OUTPUT);
    digitalWrite(LORA_CS, HIGH);
    SPI1.begin(false);
#else  // HW_SPI1_DEVICE
    SPI.setSCK(LORA_SCK);
    SPI.setTX(LORA_MOSI);
    SPI.setRX(LORA_MISO);
    SPI.begin(false);
#endif // HW_SPI1_DEVICE
#elif ARCH_PORTDUINO
    if (portduino_config.lora_spi_dev != "ch341") {
        SPI.begin();
    }
#elif !defined(ARCH_ESP32) // ARCH_RP2040
#if defined(RAK3401) || defined(RAK13302)
    pinMode(WB_IO2, OUTPUT);
    digitalWrite(WB_IO2, HIGH);
    SPI1.setPins(LORA_MISO, LORA_SCK, LORA_MOSI);
    SPI1.begin();
#else
    SPI.begin();
#endif
#else
        // ESP32
#if defined(HW_SPI1_DEVICE)
    SPI1.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    LOG_DEBUG("SPI1.begin(SCK=%d, MISO=%d, MOSI=%d, NSS=%d)", LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    SPI1.setFrequency(4000000);
#else
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    LOG_DEBUG("SPI.begin(SCK=%d, MISO=%d, MOSI=%d, NSS=%d)", LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    SPI.setFrequency(4000000);
#endif
#endif

    // setup TZ prior to time actions.
#if !MESHTASTIC_EXCLUDE_TZ
    LOG_DEBUG("Use compiled/slipstreamed %s", slipstreamTZString); // important, removing this clobbers our magic string
    if (*config.device.tzdef && config.device.tzdef[0] != 0) {
        LOG_DEBUG("Saved TZ: %s ", config.device.tzdef);
        setenv("TZ", config.device.tzdef, 1);
    } else {
        if (strncmp((const char *)slipstreamTZString, "tzpl", 4) == 0) {
            setenv("TZ", "GMT0", 1);
        } else {
            setenv("TZ", (const char *)slipstreamTZString, 1);
            strcpy(config.device.tzdef, (const char *)slipstreamTZString);
        }
    }
    tzset();
    LOG_DEBUG("Set Timezone to %s", getenv("TZ"));
#endif

    readFromRTC(); // read the main CPU RTC at first (in case we can't get GPS time)

#ifdef PIN_GPS_STANDBY
    pinMode(PIN_GPS_STANDBY, OUTPUT);
#ifdef PIN_GPS_STANDBY_INVERTED
    digitalWrite(PIN_GPS_STANDBY, 1);
#else
    digitalWrite(PIN_GPS_STANDBY, 0);
#endif
#endif

    nodeStatus->observe(&nodeDB->newStatus);

#ifdef HAS_I2S
    LOG_DEBUG("Start audio thread");
    audioThread = new AudioThread();
#endif

#ifdef HAS_UDP_MULTICAST
    LOG_DEBUG("Start multicast thread");
    udpHandler = new UdpMulticastHandler();
#ifdef ARCH_PORTDUINO
    // FIXME: portduino does not ever call onNetworkConnected so call it here because I don't know what happen if I call
    // onNetworkConnected there
    if (config.network.enabled_protocols & meshtastic_Config_NetworkConfig_ProtocolFlags_UDP_BROADCAST) {
        udpHandler->start();
    }
#endif
#endif
    service = new MeshService();
    service->init();

    // Set osk_found for trackball/encoder devices BEFORE setupModules so CannedMessageModule can detect it
#if defined(HAS_TRACKBALL) || (defined(INPUTDRIVER_ENCODER_TYPE) && INPUTDRIVER_ENCODER_TYPE == 2)
#ifndef HAS_PHYSICAL_KEYBOARD
    osk_found = true;
#endif
#endif

    // Now that the mesh service is created, create any modules
    setupModules();

#if !MESHTASTIC_EXCLUDE_I2C
    // Inform modules about I2C devices
    ScanI2CCompleted(i2cScanner.get());
    i2cScanner.reset();
#endif

#if !defined(MESHTASTIC_EXCLUDE_PKI)
    // warn the user about a low entropy key
    if (nodeDB->keyIsLowEntropy && !nodeDB->hasWarned) {
        LOG_WARN(LOW_ENTROPY_WARNING);
        meshtastic_ClientNotification *cn = clientNotificationPool.allocZeroed();
        cn->level = meshtastic_LogRecord_Level_WARNING;
        cn->time = getValidTime(RTCQualityFromNet);
        sprintf(cn->message, LOW_ENTROPY_WARNING);
        service->sendClientNotification(cn);
        nodeDB->hasWarned = true;
    }
#endif

#ifdef LED_PIN
    // Turn LED off after boot, if heartbeat by config
    if (config.device.led_heartbeat_disabled)
        digitalWrite(LED_PIN, HIGH ^ LED_STATE_ON);
#endif

// Do this after service.init (because that clears error_code)
#ifdef HAS_PMU
    if (!pmu_found)
        RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_NO_AXP192); // Record a hardware fault for missing hardware
#endif

    auto rIf = initLoRa();

    lateInitVariant(); // Do board specific init (see extra_variants/README.md for documentation)

#ifdef RF95_FAN_EN
    // Ability to disable FAN if PIN has been set with RF95_FAN_EN.
    // Make sure LoRa has been started before disabling FAN.
    if (config.lora.pa_fan_disabled)
        digitalWrite(RF95_FAN_EN, LOW ^ 0);
#endif

#ifndef ARCH_PORTDUINO

        // Initialize Wifi
#if HAS_WIFI
    initWifi();
#endif

#if HAS_ETHERNET
    // Initialize Ethernet
    initEthernet();
#endif
#endif

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_WEBSERVER
    // Start web server thread.
    webServerThread = new WebServerThread();
#endif

#ifdef ARCH_PORTDUINO
#if __has_include(<ulfius.h>)
    if (portduino_config.webserverport != -1) {
        piwebServerThread = new PiWebServerThread();
        std::atexit([] { delete piwebServerThread; });
    }
#endif
    initApiServer(TCPPort);
#endif

    // Start airtime logger thread.
    airTime = new AirTime();

    // Initialize on boot so it triggers even when initial radio chip detection fails.
    lastRxMsec = millis();

    if (!rIf)
        RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_NO_RADIO);
    else {
        // Log bit rate to debug output
        LOG_DEBUG("LoRA bitrate = %f bytes / sec", (float(meshtastic_Constants_DATA_PAYLOAD_LEN) /
                                                    (float(rIf->getPacketTime(meshtastic_Constants_DATA_PAYLOAD_LEN)))) *
                                                       1000);

        router->addInterface(std::move(rIf));
    }

    // This must be _after_ service.init because we need our preferences loaded from flash to have proper timeout values
    PowerFSM_setup(); // we will transition to ON in a couple of seconds, FIXME, only do this for cold boots, not waking from SDS
    powerFSMthread = new PowerFSMThread();

#if !HAS_TFT
    setCPUFast(false); // 80MHz is fine for our slow peripherals
#endif

#ifdef ARDUINO_ARCH_ESP32
    LOG_DEBUG("Free heap  : %7d bytes", ESP.getFreeHeap());
    LOG_DEBUG("Free PSRAM : %7d bytes", ESP.getFreePsram());
#endif

    // We manually run this to update the NodeStatus
    nodeDB->notifyObservers(true);
}

#endif
uint32_t rebootAtMsec;     // If not zero we will reboot at this time (used to reboot shortly after the update completes)
uint32_t shutdownAtMsec;   // If not zero we will shutdown at this time (used to shutdown from python or mobile client)
bool suppressRebootBanner; // If true, suppress "Rebooting..." overlay (used for OTA handoff)

// If a thread does something that might need for it to be rescheduled ASAP it can set this flag
// This will suppress the current delay and instead try to run ASAP.
bool runASAP;

// TODO find better home than main.cpp
extern meshtastic_DeviceMetadata getDeviceMetadata()
{
    meshtastic_DeviceMetadata deviceMetadata;
    strncpy(deviceMetadata.firmware_version, optstr(APP_VERSION), sizeof(deviceMetadata.firmware_version));
    deviceMetadata.device_state_version = DEVICESTATE_CUR_VER;
    deviceMetadata.canShutdown = pmu_found || HAS_CPU_SHUTDOWN;
    deviceMetadata.hasBluetooth = HAS_BLUETOOTH;
    deviceMetadata.hasWifi = HAS_WIFI;
    deviceMetadata.hasEthernet = HAS_ETHERNET;
    deviceMetadata.role = config.device.role;
    deviceMetadata.position_flags = config.position.position_flags;
    deviceMetadata.hw_model = HW_VENDOR;
    deviceMetadata.hasRemoteHardware = moduleConfig.remote_hardware.enabled;
    deviceMetadata.excluded_modules = meshtastic_ExcludedModules_EXCLUDED_NONE;
    deviceMetadata.excluded_modules |= meshtastic_ExcludedModules_REMOTEHARDWARE_CONFIG;
    deviceMetadata.excluded_modules |= meshtastic_ExcludedModules_AUDIO_CONFIG;
    deviceMetadata.excluded_modules |= meshtastic_ExcludedModules_CANNEDMSG_CONFIG;
    deviceMetadata.excluded_modules |= meshtastic_ExcludedModules_EXTNOTIF_CONFIG;
    deviceMetadata.excluded_modules |= meshtastic_ExcludedModules_DETECTIONSENSOR_CONFIG;
    deviceMetadata.excluded_modules |= meshtastic_ExcludedModules_SERIAL_CONFIG;
    deviceMetadata.excluded_modules |= meshtastic_ExcludedModules_PAXCOUNTER_CONFIG;
    deviceMetadata.excluded_modules |= meshtastic_ExcludedModules_AMBIENTLIGHTING_CONFIG;

// No bluetooth on these targets (yet):
// Pico W / 2W may get it at some point
// Portduino and ESP32-C6 are excluded because we don't have a working bluetooth stacks integrated yet.
#if defined(ARCH_RP2040) || defined(ARCH_PORTDUINO) || defined(ARCH_STM32WL) || defined(CONFIG_IDF_TARGET_ESP32C6)
    deviceMetadata.excluded_modules |= meshtastic_ExcludedModules_BLUETOOTH_CONFIG;
#endif

#if defined(ARCH_NRF52) && !HAS_ETHERNET // nrf52 doesn't have network unless it's a RAK ethernet gateway currently
    deviceMetadata.excluded_modules |= meshtastic_ExcludedModules_NETWORK_CONFIG; // No network on nRF52
#elif defined(ARCH_RP2040) && !HAS_WIFI && !HAS_ETHERNET
    deviceMetadata.excluded_modules |= meshtastic_ExcludedModules_NETWORK_CONFIG; // No network on RP2040
#endif

#if !(MESHTASTIC_EXCLUDE_PKI)
    deviceMetadata.hasPKC = true;
#endif
    return deviceMetadata;
}

#if !MESHTASTIC_EXCLUDE_I2C
void scannerToSensorsMap(const std::unique_ptr<ScanI2CTwoWire> &i2cScanner, ScanI2C::DeviceType deviceType,
                         meshtastic_TelemetrySensorType sensorType)
{
    auto found = i2cScanner->find(deviceType);
    if (found.type != ScanI2C::DeviceType::NONE) {
        nodeTelemetrySensorsMap[sensorType].first = found.address.address;
        nodeTelemetrySensorsMap[sensorType].second = i2cScanner->fetchI2CBus(found.address);
    }
}
#endif

#ifndef PIO_UNIT_TESTING
void loop()
{
    runASAP = false;

#if HAS_EXT_WATCHDOG && defined(EXT_WATCHDOG_TRIGGER)
    // Reset external watchdog via pin change
    digitalWrite(EXT_WATCHDOG_TRIGGER, !digitalRead(EXT_WATCHDOG_TRIGGER));
#endif

    // Radio chip watchdog.
    // Check last RX periode if radio chip is still alive.
    // Timeout one hour. Plenty of time to receive at least "something".
    if (!Throttle::isWithinTimespanMs(lastRxMsec, MS_IN_HOUR) && rebootAtMsec == 0) {
        rebootAtMsec = millis() + 3000; // Dead. Reboot in 3s
    }

#ifdef ARCH_ESP32
    esp32Loop();
#endif
#ifdef ARCH_NRF52
    nrf52Loop();
#endif
    power->powerCommandsCheck();

#ifdef DEBUG_STACK
    static uint32_t lastPrint = 0;
    if (!Throttle::isWithinTimespanMs(lastPrint, 10 * 1000L)) {
        lastPrint = millis();
        meshtastic::printThreadInfo("main");
    }
#endif

    service->loop();
    long delayMsec = mainController.runOrDelay();

    // We want to sleep as long as possible here - because it saves power
    if (!runASAP && loopCanSleep()) {
#ifdef DEBUG_LOOP_TIMING
        LOG_DEBUG("main loop delay: %d", delayMsec);
#endif
        mainDelay.delay(delayMsec);
    }
}
#endif
