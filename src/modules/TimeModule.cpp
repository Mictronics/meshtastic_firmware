#include "TimeModule.h"
#include "RTC.h"
#include "main.h"
#include <Throttle.h>

TimeModule::TimeModule() : ProtobufModule("time", meshtastic_PortNum_POSITION_APP, &meshtastic_Position_msg) {}

bool TimeModule::handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_Position *pptr)
{
    auto p = *pptr;
    // Log packet size and data fields
    LOG_DEBUG("TIME node=%08x time=%d", getFrom(&mp), p.time);

    if (p.time && channels.getByIndex(mp.channel).role == meshtastic_Channel_Role_PRIMARY) {
        bool force = false;

#ifdef T_WATCH_S3
        // The T-Watch appears to "pause" its RTC when shut down, such that the time it reads upon powering on is the same as when
        // it was shut down. So we need to force the update here, since otherwise RTC::perhapsSetRTC will ignore it because it
        // will always be an equivalent or lesser RTCQuality (RTCQualityNTP or RTCQualityNet).
        force = true;
#endif
        // Set from phone RTC Quality to RTCQualityNTP since it should be approximately so
        trySetRtc(p, false, force);
    }

    return true; // No further processing needed
}

void TimeModule::trySetRtc(meshtastic_Position p, bool isLocal, bool forceUpdate)
{
    if (hasQualityTimesource() && !isLocal) {
        LOG_DEBUG("Ignore time from mesh because we have a GPS, RTC, or Phone/NTP time source in the past day");
        return;
    }
    if (!isLocal && p.location_source < meshtastic_Position_LocSource_LOC_INTERNAL) {
        LOG_DEBUG("Ignore time from mesh because it has a unknown or manual source");
        return;
    }
    struct timeval tv;
    uint32_t secs = p.time;

    tv.tv_sec = secs;
    tv.tv_usec = 0;
    perhapsSetRTC(isLocal ? RTCQualityNTP : RTCQualityFromNet, &tv, forceUpdate);
}

bool TimeModule::hasQualityTimesource()
{
    bool setFromPhoneOrNtpToday =
        lastSetFromPhoneNtpOrGps == 0 ? false : Throttle::isWithinTimespanMs(lastSetFromPhoneNtpOrGps, SEC_PER_DAY * 1000UL);
    bool hasRtc = (rtc_found.address != ScanI2C::ADDRESS_NONE.address);
    return hasRtc || setFromPhoneOrNtpToday;
}
