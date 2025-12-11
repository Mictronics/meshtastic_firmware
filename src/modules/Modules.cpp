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

#include "modules/TimeModule.h"

#include "modules/RoutingModule.h"
#if !MESHTASTIC_EXCLUDE_TEXTMESSAGE
#include "modules/TextMessageModule.h"
#endif
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

    new TimeModule();

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
