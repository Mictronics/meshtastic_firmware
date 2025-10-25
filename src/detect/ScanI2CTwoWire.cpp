#include "ScanI2CTwoWire.h"

#if !MESHTASTIC_EXCLUDE_I2C

#include "concurrency/LockGuard.h"
#if defined(ARCH_PORTDUINO)
#include "linux/LinuxHardwareI2C.h"
#endif
#if !defined(ARCH_PORTDUINO) && !defined(ARCH_STM32WL)
#include "meshUtils.h" // vformat
#endif

bool in_array(uint8_t *array, int size, uint8_t lookfor)
{
    int i;
    for (i = 0; i < size; i++)
        if (lookfor == array[i])
            return true;
    return false;
}

ScanI2C::FoundDevice ScanI2CTwoWire::find(ScanI2C::DeviceType type) const
{
    concurrency::LockGuard guard((concurrency::Lock *)&lock);

    return exists(type) ? ScanI2C::FoundDevice(type, deviceAddresses.at(type)) : DEVICE_NONE;
}

bool ScanI2CTwoWire::exists(ScanI2C::DeviceType type) const
{
    return deviceAddresses.find(type) != deviceAddresses.end();
}

ScanI2C::FoundDevice ScanI2CTwoWire::firstOfOrNONE(size_t count, DeviceType types[]) const
{
    concurrency::LockGuard guard((concurrency::Lock *)&lock);

    for (size_t k = 0; k < count; k++) {
        ScanI2C::DeviceType current = types[k];

        if (exists(current)) {
            return ScanI2C::FoundDevice(current, deviceAddresses.at(current));
        }
    }

    return DEVICE_NONE;
}

uint16_t ScanI2CTwoWire::getRegisterValue(const ScanI2CTwoWire::RegisterLocation &registerLocation,
                                          ScanI2CTwoWire::ResponseWidth responseWidth, bool zeropad = false) const
{
    uint16_t value = 0x00;
    TwoWire *i2cBus = fetchI2CBus(registerLocation.i2cAddress);

    i2cBus->beginTransmission(registerLocation.i2cAddress.address);
    i2cBus->write(registerLocation.registerAddress);
    if (zeropad) {
        // Lark Commands need the argument list length in 2 bytes.
        i2cBus->write((int)0);
        i2cBus->write((int)0);
    }
    i2cBus->endTransmission();
    delay(20);
    i2cBus->requestFrom(registerLocation.i2cAddress.address, responseWidth);
    if (i2cBus->available() > 1) {
        // Read MSB, then LSB
        value = (uint16_t)i2cBus->read() << 8;
        value |= i2cBus->read();
    } else if (i2cBus->available()) {
        value = i2cBus->read();
    }
    // Drain excess bytes
    for (uint8_t i = 0; i < responseWidth - 1; i++) {
        if (i2cBus->available())
            i2cBus->read();
    }
    return value;
}

#define SCAN_SIMPLE_CASE(ADDR, T, ...)                                                                                           \
    case ADDR:                                                                                                                   \
        logFoundDevice(__VA_ARGS__);                                                                                             \
        type = T;                                                                                                                \
        break;

void ScanI2CTwoWire::scanPort(I2CPort port, uint8_t *address, uint8_t asize)
{
    concurrency::LockGuard guard((concurrency::Lock *)&lock);

    LOG_DEBUG("Scan for I2C devices on port %d", port);

    uint8_t err;

    DeviceAddress addr(port, 0x00);

    uint16_t registerValue = 0x00;
    ScanI2C::DeviceType type;
    TwoWire *i2cBus;
#ifdef RV3028_RTC
    Melopero_RV3028 rtc;
#endif

#if WIRE_INTERFACES_COUNT == 2
    if (port == I2CPort::WIRE1) {
        i2cBus = &Wire1;
    } else {
#endif
        i2cBus = &Wire;
#if WIRE_INTERFACES_COUNT == 2
    }
#endif

    // We only need to scan 112 addresses, the rest is reserved for special purposes
    // 0x00 General Call
    // 0x01 CBUS addresses
    // 0x02 Reserved for different bus formats
    // 0x03 Reserved for future purposes
    // 0x04-0x07 High Speed Master Code
    // 0x78-0x7B 10-bit slave addressing
    // 0x7C-0x7F Reserved for future purposes

    for (addr.address = 8; addr.address < 120; addr.address++) {
        if (asize != 0) {
            if (!in_array(address, asize, (uint8_t)addr.address))
                continue;
            LOG_DEBUG("Scan address 0x%x", (uint8_t)addr.address);
        }
        i2cBus->beginTransmission(addr.address);
#ifdef ARCH_PORTDUINO
        err = 2;
        if ((addr.address >= 0x30 && addr.address <= 0x37) || (addr.address >= 0x50 && addr.address <= 0x5F)) {
            if (i2cBus->read() != -1)
                err = 0;
        } else {
            err = i2cBus->writeQuick((uint8_t)0);
        }
        if (err != 0)
            err = 2;
#else
        err = i2cBus->endTransmission();
#endif
        type = NONE;
        if (err == 0) {
            switch (addr.address) {
#ifdef RV3028_RTC
            case RV3028_RTC:
                // foundDevices[addr] = RTC_RV3028;
                type = RTC_RV3028;
                logFoundDevice("RV3028", (uint8_t)addr.address);
                rtc.initI2C(*i2cBus);
                // Update RTC EEPROM settings, if necessary
                if (rtc.readEEPROMRegister(0x35) != 0x07) {
                    rtc.writeEEPROMRegister(0x35, 0x07); // no Clkout
                }
                if (rtc.readEEPROMRegister(0x37) != 0xB4) {
                    rtc.writeEEPROMRegister(0x37, 0xB4);
                }
                break;
#endif

#ifdef PCF8563_RTC
                SCAN_SIMPLE_CASE(PCF8563_RTC, RTC_PCF8563, "PCF8563", (uint8_t)addr.address)
#endif
            case BME_ADDR:
            case BME_ADDR_ALTERNATE:
                registerValue = getRegisterValue(ScanI2CTwoWire::RegisterLocation(addr, 0xD0), 1); // GET_ID
                switch (registerValue) {
                case 0x61:
                    logFoundDevice("BME680", (uint8_t)addr.address);
                    type = BME_680;
                    break;
                case 0x60:
                    logFoundDevice("BME280", (uint8_t)addr.address);
                    type = BME_280;
                    break;
                case 0x55:
                    logFoundDevice("BMP085/BMP180", (uint8_t)addr.address);
                    type = BMP_085;
                    break;
                default:
                    registerValue = getRegisterValue(ScanI2CTwoWire::RegisterLocation(addr, 0x00), 1); // GET_ID
                    switch (registerValue) {
                    case 0x50: // BMP-388 should be 0x50
                        logFoundDevice("BMP-388", (uint8_t)addr.address);
                        type = BMP_3XX;
                        break;
                    case 0x60: // BMP-390 should be 0x60
                        logFoundDevice("BMP-390", (uint8_t)addr.address);
                        type = BMP_3XX;
                        break;
                    case 0x58: // BMP-280 should be 0x58
                    default:
                        logFoundDevice("BMP-280", (uint8_t)addr.address);
                        type = BMP_280;
                        break;
                    }
                    break;
                }
                break;

#if !defined(M5STACK_UNITC6L)
            case INA_ADDR:
            case INA_ADDR_ALTERNATE:
            case INA_ADDR_WAVESHARE_UPS:
                registerValue = getRegisterValue(ScanI2CTwoWire::RegisterLocation(addr, 0xFE), 2);
                LOG_DEBUG("Register MFG_UID: 0x%x", registerValue);
                if (registerValue == 0x5449) {
                    registerValue = getRegisterValue(ScanI2CTwoWire::RegisterLocation(addr, 0xFF), 2);
                    LOG_DEBUG("Register DIE_UID: 0x%x", registerValue);

                    if (registerValue == 0x2260) {
                        logFoundDevice("INA226", (uint8_t)addr.address);
                        type = INA226;
                    } else {
                        logFoundDevice("INA260", (uint8_t)addr.address);
                        type = INA260;
                    }
                } else { // Assume INA219 if INA260 ID is not found
                    logFoundDevice("INA219", (uint8_t)addr.address);
                    type = INA219;
                }
                break;
            case INA3221_ADDR:
                registerValue = getRegisterValue(ScanI2CTwoWire::RegisterLocation(addr, 0xFE), 2);
                LOG_DEBUG("Register MFG_UID FE: 0x%x", registerValue);
                if (registerValue == 0x5449) {
                    logFoundDevice("INA3221", (uint8_t)addr.address);
                    type = INA3221;
                }
                break;
#endif
            default:
                LOG_INFO("Device found at address 0x%x was not able to be enumerated", (uint8_t)addr.address);
            }
        } else if (err == 4) {
            LOG_ERROR("Unknown error at address 0x%x", (uint8_t)addr.address);
        }

        // Check if a type was found for the enumerated device - save, if so
        if (type != NONE) {
            deviceAddresses[type] = addr;
            foundDevices[addr] = type;
        }
    }
}

void ScanI2CTwoWire::scanPort(I2CPort port)
{
    scanPort(port, nullptr, 0);
}

TwoWire *ScanI2CTwoWire::fetchI2CBus(ScanI2C::DeviceAddress address)
{
    if (address.port == ScanI2C::I2CPort::WIRE) {
        return &Wire;
    } else {
#if WIRE_INTERFACES_COUNT == 2
        return &Wire1;
#else
        return &Wire;
#endif
    }
}

size_t ScanI2CTwoWire::countDevices() const
{
    return foundDevices.size();
}

void ScanI2CTwoWire::logFoundDevice(const char *device, uint8_t address)
{
    LOG_INFO("%s found at address 0x%x", device, address);
}
#endif
