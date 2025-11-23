#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_PKI
#include "KeyVerificationModule.h"
#endif
#if !MESHTASTIC_EXCLUDE_ADMIN
#include "modules/AdminModule.h"
#endif
#if !MESHTASTIC_EXCLUDE_NODEINFO
#include "modules/NodeInfoModule.h"
#endif
#if !MESHTASTIC_EXCLUDE_GPS
#include "modules/PositionModule.h"
#else
#include "modules/TimeModule.h"
#endif
#include "modules/RoutingModule.h"
#include "modules/TextMessageModule.h"
#if ARCH_PORTDUINO
#include "modules/Telemetry/HostMetrics.h"
#endif
#if HAS_TELEMETRY
#include "modules/Telemetry/DeviceTelemetry.h"
#endif
#if HAS_SENSOR && !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR
#include "main.h"
#include "modules/Telemetry/EnvironmentTelemetry.h"
#include "modules/Telemetry/Sensor/TelemetrySensor.h"
#endif
#if HAS_TELEMETRY && !MESHTASTIC_EXCLUDE_POWER_TELEMETRY
#include "modules/Telemetry/PowerTelemetry.h"
#endif

/**
 * Create module instances here.  If you are adding a new module, you must 'new' it here (or somewhere else)
 */
void setupModules()
{
#if !MESHTASTIC_EXCLUDE_ADMIN
    adminModule = new AdminModule();
#endif
#if !MESHTASTIC_EXCLUDE_NODEINFO
    nodeInfoModule = new NodeInfoModule();
#endif
#if !MESHTASTIC_EXCLUDE_GPS
    positionModule = new PositionModule();
#else
    new TimeModule();
#endif
#if !MESHTASTIC_EXCLUDE_TEXTMESSAGE
    textMessageModule = new TextMessageModule();
#endif
#if !MESHTASTIC_EXCLUDE_PKI
    keyVerificationModule = new KeyVerificationModule();
#endif
    // Note: if the rest of meshtastic doesn't need to explicitly use your module, you do not need to assign the instance
    // to a global variable.
#if ARCH_PORTDUINO
    new HostMetricsModule();
#endif
#if HAS_TELEMETRY
    new DeviceTelemetryModule();
#endif
#if HAS_TELEMETRY && HAS_SENSOR && !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR
    new EnvironmentTelemetryModule();
#endif
#if HAS_TELEMETRY && !MESHTASTIC_EXCLUDE_POWER_TELEMETRY && !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR
    new PowerTelemetryModule();
#endif
    // NOTE! This module must be added LAST because it likes to check for replies from other modules and avoid sending extra
    // acks
    routingModule = new RoutingModule();
}
