#include "Default.h"

#include "meshUtils.h"

// Convert seconds to ms, clamping at INT32_MAX (~24.86 days)
static inline uint32_t secondsToMsClamped(uint32_t secs)
{
    constexpr uint32_t MAX_MS = static_cast<uint32_t>(INT32_MAX);
    return (secs > MAX_MS / 1000U) ? MAX_MS : secs * 1000U;
}

uint32_t Default::getConfiguredOrDefaultMs(uint32_t configuredInterval, uint32_t defaultInterval)
{
    return secondsToMsClamped(configuredInterval > 0 ? configuredInterval : defaultInterval);
}

uint32_t Default::getConfiguredOrDefaultMs(uint32_t configuredInterval)
{
    return secondsToMsClamped(configuredInterval > 0 ? configuredInterval : default_broadcast_interval_secs);
}

uint32_t Default::getConfiguredOrDefault(uint32_t configured, uint32_t defaultValue)
{
    if (configured > 0)
        return configured;
    return defaultValue;
}
/**
 * Calculates the scaled value of the configured or default value in ms based on the number of online nodes.
 *
 * For example a default of 30 minutes (1800 seconds * 1000) would yield:
 *   45 nodes = 2475 * 1000
 *   60 nodes = 4500 * 1000
 *   75 nodes = 6525 * 1000
 *   90 nodes = 8550 * 1000
 * @param configured The configured value.
 * @param defaultValue The default value.
 * @param numOnlineNodes The number of online nodes.
 * @return The scaled value of the configured or default value.
 */
uint32_t Default::getConfiguredOrDefaultMsScaled(uint32_t configured, uint32_t defaultValue, uint32_t numOnlineNodes)
{
    return getConfiguredOrDefaultMs(configured, defaultValue);
}

uint32_t Default::getConfiguredOrMinimumValue(uint32_t configured, uint32_t minValue)
{
    // If zero, intervals should be coalesced later by getConfiguredOrDefault... methods
    if (configured == 0)
        return configured;

    return configured < minValue ? minValue : configured;
}

uint8_t Default::getConfiguredOrDefaultHopLimit(uint8_t configured)
{
#if USERPREFS_EVENT_MODE
    return (configured > HOP_RELIABLE) ? HOP_RELIABLE : config.lora.hop_limit;
#else
    return (configured >= HOP_MAX) ? HOP_MAX : config.lora.hop_limit;
#endif
}
