#include "INA219Sensor.h"
#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "TelemetrySensor.h"
#include "configuration.h"
#include <Adafruit_INA219.h>

INA219Sensor::INA219Sensor() : TelemetrySensor(meshtastic_TelemetrySensorType_INA219, "INA219") {}

int32_t INA219Sensor::runOnce()
{
    LOG_INFO("Init sensor: %s\n", sensorName);
    if (!hasSensor()) {
        return DEFAULT_SENSOR_MINIMUM_WAIT_TIME_BETWEEN_READS;
    }
    if (!ina219.success()) {
        ina219 = Adafruit_INA219(nodeTelemetrySensorsMap[sensorType].first);
        status = ina219.begin(nodeTelemetrySensorsMap[sensorType].second);
    } else {
        status = ina219.success();
    }
    return initI2CSensor();
}

void INA219Sensor::setup() {}

bool INA219Sensor::getMetrics(meshtastic_Telemetry *measurement)
{
    measurement->variant.environment_metrics.voltage = ina219.getBusVoltage_V();
#ifdef USE_WAVESHARE_PICO_UPS
    // Waveshare PICO-UPS-A/B uses a 0.01 Ohm shunt
    measurement->variant.environment_metrics.current = ina219.getCurrent_mA() * 10.0f;
#else
    measurement->variant.environment_metrics.current = ina219.getCurrent_mA();
#endif
    return true;
}

uint16_t INA219Sensor::getBusVoltageMv()
{
    return lround(ina219.getBusVoltage_V() * 1000);
}