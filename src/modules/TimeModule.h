#pragma once
#include "Default.h"
#include "ProtobufModule.h"

/**
 * Time module for receiving the time from the mesh
 * when position module (GPS) is excluded from build.
 */
class TimeModule : public ProtobufModule<meshtastic_Position>
{
  public:
    /** Constructor
     * name is for debugging output
     */
    TimeModule();

  protected:
    /** Called to handle a particular incoming message
    @return true if you've guaranteed you've handled this message and no other handlers should be considered for it
    */
    virtual bool handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_Position *p) override;

  private:
    void trySetRtc(meshtastic_Position p, bool isLocal, bool forceUpdate = false);
    bool hasQualityTimesource();
};

extern TimeModule timeModule;