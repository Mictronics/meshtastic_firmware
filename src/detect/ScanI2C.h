#pragma once

#include <stddef.h>
#include <stdint.h>

class ScanI2C
{
  public:
    typedef enum DeviceType {
        NONE,
        RTC_RV3028,
        RTC_PCF8563,
        RTC_RX8130CE,
        BME_680,
        BME_280,
        BMP_280,
        BMP_085,
        BMP_3XX,
        INA260,
        INA219,
        INA3221,
        INA226,
    } DeviceType;

    // typedef uint8_t DeviceAddress;
    typedef enum I2CPort {
        NO_I2C,
        WIRE,
        WIRE1,
    } I2CPort;

    typedef struct DeviceAddress {
        // set default values for ADDRESS_NONE
        I2CPort port = I2CPort::NO_I2C;
        uint8_t address = 0;

        explicit DeviceAddress(I2CPort port, uint8_t address);
        DeviceAddress();

        bool operator<(const DeviceAddress &other) const;
    } DeviceAddress;

    static const DeviceAddress ADDRESS_NONE;

    typedef uint8_t RegisterAddress;

    typedef struct FoundDevice {
        DeviceType type;
        DeviceAddress address;

        explicit FoundDevice(DeviceType = DeviceType::NONE, DeviceAddress = ADDRESS_NONE);
    } FoundDevice;

    static const FoundDevice DEVICE_NONE;

  public:
    ScanI2C();

    virtual void scanPort(ScanI2C::I2CPort);
    virtual void scanPort(ScanI2C::I2CPort, uint8_t *, uint8_t);

    FoundDevice firstRTC() const;

    virtual FoundDevice find(DeviceType) const;

    virtual bool exists(DeviceType) const;

    virtual size_t countDevices() const;

  protected:
    virtual FoundDevice firstOfOrNONE(size_t, DeviceType[]) const;
};