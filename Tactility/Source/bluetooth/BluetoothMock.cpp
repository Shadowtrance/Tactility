#ifdef ESP_PLATFORM
#include <sdkconfig.h>
#endif

#if !defined(CONFIG_BT_NIMBLE_ENABLED)

#include <Tactility/bluetooth/Bluetooth.h>
#include <Tactility/PubSub.h>

namespace tt::bluetooth {

static std::shared_ptr<PubSub<BtEvent>> bt_pubsub = std::make_shared<PubSub<BtEvent>>();

std::shared_ptr<PubSub<BtEvent>> getPubsub() { return bt_pubsub; }

const char* radioStateToString(RadioState state) {
    switch (state) {
        using enum RadioState;
        case Off:        return "Off";
        case OnPending:  return "OnPending";
        case On:         return "On";
        case OffPending: return "OffPending";
    }
    return "Unknown";
}

RadioState getRadioState()                    { return RadioState::Off; }
void       setEnabled(bool /*enabled*/)       {}
void       scanStart()                        {}
void       scanStop()                         {}
bool       isScanning()                       { return false; }

std::vector<PeerRecord> getScanResults()      { return {}; }
std::vector<PeerRecord> getPairedPeers()      { return {}; }

void pair(const std::array<uint8_t, 6>& /*addr*/)              {}
void unpair(const std::array<uint8_t, 6>& /*addr*/)            {}
void connect(const std::array<uint8_t, 6>& /*addr*/, int /*profileId*/)    {}
void disconnect(const std::array<uint8_t, 6>& /*addr*/, int /*profileId*/) {}

bool isProfileSupported(int /*profileId*/)    { return false; }

void hidHostConnect(const std::array<uint8_t, 6>& /*addr*/) {}
void hidHostDisconnect()                      {}
bool hidHostIsConnected()                     { return false; }

bool hidDeviceStart(uint16_t /*appearance*/)  { return false; }
void hidDeviceStop()                          {}
bool hidDeviceIsConnected()                   { return false; }

bool hidSendKeyboard(const uint8_t /*report*/[8]) { return false; }
bool hidSendConsumer(const uint8_t /*report*/[2]) { return false; }
bool hidSendMouse(const uint8_t /*report*/[4])    { return false; }
bool hidSendGamepad(const uint8_t /*report*/[8])  { return false; }

bool   sppStart()                                          { return false; }
void   sppStop()                                           {}
bool   sppWrite(const uint8_t* /*data*/, size_t /*len*/)  { return false; }
size_t sppRead(uint8_t* /*data*/, size_t /*max_len*/)     { return 0; }
bool   sppIsConnected()                                    { return false; }

bool   midiStart()                                         { return false; }
void   midiStop()                                          {}
bool   midiSend(const uint8_t* /*msg*/, size_t /*len*/)   { return false; }
bool   midiIsConnected()                                   { return false; }
size_t midiRead(uint8_t* /*data*/, size_t /*max_len*/)    { return 0; }

void systemStart() {}

} // namespace tt::bluetooth

#endif // !CONFIG_BT_NIMBLE_ENABLED
