#pragma once
#include "Default.h"
#include "ProtobufModule.h"
#include "concurrency/OSThread.h"

/**
 * Position module for sending/receiving positions into the mesh
 */
class PositionModule : public ProtobufModule<meshtastic_Position>, private concurrency::OSThread
{
    CallbackObserver<PositionModule, const meshtastic::Status *> nodeStatusObserver =
        CallbackObserver<PositionModule, const meshtastic::Status *>(this, &PositionModule::handleStatusUpdate);

    /// The id of the last packet we sent, to allow us to cancel it if we make something fresher
    PacketId prevPacketId = 0;

    /// We limit our GPS broadcasts to a max rate
    uint32_t lastGpsSend = 0;

    // Store the latest good lat / long
    int32_t lastGpsLatitude = 0;
    int32_t lastGpsLongitude = 0;

    /// We force a rebroadcast if the radio settings change
    uint32_t currentGeneration = 0;

  public:
    /** Constructor
     * name is for debugging output
     */
    PositionModule();

    /**
     * Send our position into the mesh
     */
    void sendOurPosition(NodeNum dest, bool wantReplies = false, uint8_t channel = 0);
    void sendOurPosition();

    void handleNewPosition();

  protected:
    /** Called to handle a particular incoming message

    @return true if you've guaranteed you've handled this message and no other handlers should be considered for it
    */
    virtual bool handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_Position *p) override;

    virtual void alterReceivedProtobuf(meshtastic_MeshPacket &mp, meshtastic_Position *p) override;

    /** Messages can be received that have the want_response bit set.  If set, this callback will be invoked
     * so that subclasses can (optionally) send a response back to the original sender.  */
    virtual meshtastic_MeshPacket *allocReply() override;

    /** Does our periodic broadcast */
    virtual int32_t runOnce() override;

  private:
    meshtastic_MeshPacket *allocPositionPacket();
    struct SmartPosition getDistanceTraveledSinceLastSend(meshtastic_PositionLite currentPosition);
    meshtastic_MeshPacket *allocAtakPli();
    void trySetRtc(meshtastic_Position p, bool isLocal, bool forceUpdate = false);
    uint32_t precision;
    void sendLostAndFoundText();
    bool hasQualityTimesource();
    bool hasGPS();
    uint32_t lastSentReply = 0; // Last time we sent a position reply (used for reply throttling only)

#if USERPREFS_EVENT_MODE
    // In event mode we want to prevent excessive position broadcasts
    // we set the minimum interval to 5m
    const uint32_t minimumTimeThreshold =
        max(300000, Default::getConfiguredOrDefaultMs(config.position.broadcast_smart_minimum_interval_secs, 30));
#else
    const uint32_t minimumTimeThreshold =
        Default::getConfiguredOrDefaultMs(config.position.broadcast_smart_minimum_interval_secs, 30);
#endif
};

struct SmartPosition {
    float distanceTraveled;
    uint32_t distanceThreshold;
    bool hasTraveledOverThreshold;
};

extern PositionModule *positionModule;