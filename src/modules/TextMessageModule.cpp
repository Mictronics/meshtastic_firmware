#if !MESHTASTIC_EXCLUDE_TEXTMESSAGE
#include "TextMessageModule.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "PowerFSM.h"
#include "configuration.h"
#include "main.h"

TextMessageModule *textMessageModule;

ProcessMessage TextMessageModule::handleReceived(const meshtastic_MeshPacket &mp)
{
#if defined(DEBUG_PORT)
    auto &p = mp.decoded;
    LOG_INFO("Received text msg from=0x%0x, id=0x%x, msg=%.*s", mp.from, mp.id, p.payload.size, p.payload.bytes);
#endif
    // add packet ID to the rolling list of packets
    textPacketList[textPacketListIndex] = mp.id;
    textPacketListIndex = (textPacketListIndex + 1) % TEXT_PACKET_LIST_SIZE;

    // We only store/display messages destined for us.
    devicestate.rx_text_message = mp;
    devicestate.has_rx_text_message = true;
    // Notify any observers (e.g. external modules that care about packets)
    notifyObservers(&mp);
    return ProcessMessage::CONTINUE; // Let others look at this message also if they want
}

bool TextMessageModule::wantPacket(const meshtastic_MeshPacket *p)
{
    return MeshService::isTextPayload(p);
}

bool TextMessageModule::recentlySeen(uint32_t id)
{
    for (size_t i = 0; i < TEXT_PACKET_LIST_SIZE; i++) {
        if (textPacketList[i] != 0 && textPacketList[i] == id) {
            return true;
        }
    }
    return false;
}
#endif