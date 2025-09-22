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
#endif
#include "modules/RoutingModule.h"
#include "modules/TextMessageModule.h"
#if !MESHTASTIC_EXCLUDE_TRACEROUTE
#include "modules/TraceRouteModule.h"
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
#if !MESHTASTIC_EXCLUDE_GENERIC_THREAD_MODULE
#include "modules/GenericThreadModule.h"
#endif

/**
 * Create module instances here.  If you are adding a new module, you must 'new' it here (or somewhere else)
 */
void setupModules()
{
    if (config.device.role != meshtastic_Config_DeviceConfig_Role_REPEATER) {
#if !MESHTASTIC_EXCLUDE_ADMIN
        adminModule = new AdminModule();
#endif
#if !MESHTASTIC_EXCLUDE_NODEINFO
        nodeInfoModule = new NodeInfoModule();
#endif
#if !MESHTASTIC_EXCLUDE_GPS
        positionModule = new PositionModule();
#endif
#if !MESHTASTIC_EXCLUDE_TEXTMESSAGE
        textMessageModule = new TextMessageModule();
#endif
#if !MESHTASTIC_EXCLUDE_TRACEROUTE
        traceRouteModule = new TraceRouteModule();
#endif
#if !MESHTASTIC_EXCLUDE_PKI
        keyVerificationModule = new KeyVerificationModule();
#endif
#if !MESHTASTIC_EXCLUDE_GENERIC_THREAD_MODULE
        new GenericThreadModule();
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
    } else {
        // Configure repeater role
#if !MESHTASTIC_EXCLUDE_ADMIN
        adminModule = new AdminModule();
#endif
#if HAS_TELEMETRY
        new DeviceTelemetryModule();
#endif

#if !MESHTASTIC_EXCLUDE_TRACEROUTE
        traceRouteModule = new TraceRouteModule();
#endif
    }
    // NOTE! This module must be added LAST because it likes to check for replies from other modules and avoid sending extra
    // acks
    routingModule = new RoutingModule();
}
