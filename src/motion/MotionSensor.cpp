#include "MotionSensor.h"
#include "graphics/draw/CompassRenderer.h"

#if !defined(ARCH_STM32WL) && !MESHTASTIC_EXCLUDE_I2C

char timeRemainingBuffer[12];

// screen is defined in main.cpp
extern graphics::Screen *screen;

MotionSensor::MotionSensor(ScanI2C::FoundDevice foundDevice)
{
    device.address.address = foundDevice.address.address;
    device.address.port = foundDevice.address.port;
    device.type = foundDevice.type;
    LOG_DEBUG("Motion MotionSensor port: %s address: 0x%x type: %d", devicePort() == ScanI2C::I2CPort::WIRE1 ? "Wire1" : "Wire",
              (uint8_t)deviceAddress(), deviceType());
}

ScanI2C::DeviceType MotionSensor::deviceType()
{
    return device.type;
}

uint8_t MotionSensor::deviceAddress()
{
    return device.address.address;
}

ScanI2C::I2CPort MotionSensor::devicePort()
{
    return device.address.port;
}

#if !MESHTASTIC_EXCLUDE_POWER_FSM
void MotionSensor::wakeScreen()
{
    if (powerFSM.getState() == &stateDARK) {
        LOG_DEBUG("Motion wakeScreen detected");
        powerFSM.trigger(EVENT_INPUT);
    }
}

void MotionSensor::buttonPress()
{
    LOG_DEBUG("Motion buttonPress detected");
    powerFSM.trigger(EVENT_PRESS);
}

#else

void MotionSensor::wakeScreen() {}

void MotionSensor::buttonPress() {}

#endif

#endif