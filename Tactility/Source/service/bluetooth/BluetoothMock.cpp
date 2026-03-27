#ifdef ESP_PLATFORM
#include <sdkconfig.h>
#endif

#if not defined(CONFIG_BT_NIMBLE_ENABLED)

#include <Tactility/service/bluetooth/Bluetooth.h>

#include <Tactility/PubSub.h>
#include <Tactility/RecursiveMutex.h>
#include <tactility/check.h>
#include <Tactility/service/Service.h>
#include <Tactility/service/ServiceManifest.h>

namespace tt::service::bluetooth {

struct Bluetooth {
    /** @brief Locking mechanism for modifying the Bluetooth instance */
    RecursiveMutex mutex;
    /** @brief The public event bus */
    std::shared_ptr<PubSub<BtEvent>> pubsub = std::make_shared<PubSub<BtEvent>>();
    RadioState radioState = RadioState::On;
    bool scanActive = false;
};

static Bluetooth* bt = nullptr;

// region Public functions

std::shared_ptr<PubSub<BtEvent>> getPubsub() {
    assert(bt);
    return bt->pubsub;
}

RadioState getRadioState() {
    assert(bt);
    return bt->radioState;
}

void setEnabled(bool enabled) {
    assert(bt);
    bt->radioState = enabled ? RadioState::On : RadioState::Off;
}

void scanStart() {
    assert(bt);
    bt->scanActive = true;
}

void scanStop() {
    assert(bt);
    bt->scanActive = false;
}

bool isScanning() {
    assert(bt);
    return bt->scanActive;
}

std::vector<PeerRecord> getScanResults() {
    assert(bt);
    std::vector<PeerRecord> records;
    records.push_back(PeerRecord {
        .addr = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01 },
        .name = "BLE Keyboard",
        .rssi = -45,
        .paired = false,
        .connected = false,
    });
    records.push_back(PeerRecord {
        .addr = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x02 },
        .name = "MIDI Controller",
        .rssi = -60,
        .paired = false,
        .connected = false,
    });
    records.push_back(PeerRecord {
        .addr = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x03 },
        .name = "BLE Device",
        .rssi = -75,
        .paired = false,
        .connected = false,
    });
    return records;
}

std::vector<PeerRecord> getPairedPeers() {
    assert(bt);
    return {};
}

void pair(const std::array<uint8_t, 6>& addr) {
    assert(bt);
    // TODO: implement
}

void unpair(const std::array<uint8_t, 6>& addr) {
    assert(bt);
    // TODO: implement
}

void connect(const std::array<uint8_t, 6>& addr, int profileId) {
    assert(bt);
    // TODO: implement
}

void disconnect(const std::array<uint8_t, 6>& addr, int profileId) {
    assert(bt);
    // TODO: implement
}

bool isProfileSupported(int profileId) {
    return false;
}

bool sppStart()                                  { return false; }
void sppStop()                                   {}
bool sppWrite(const uint8_t*, size_t)            { return false; }
size_t sppRead(uint8_t*, size_t)                 { return 0; }
bool sppIsConnected()                            { return false; }

bool midiStart()                                 { return false; }
void midiStop()                                  {}
bool midiSend(const uint8_t*, size_t)            { return false; }
size_t midiRead(uint8_t*, size_t)                { return 0; }
bool midiIsConnected()                           { return false; }

void hidHostConnect(const std::array<uint8_t, 6>&) {}
void hidHostDisconnect()                         {}
bool hidHostIsConnected()                        { return false; }

bool hidDeviceStart(uint16_t /*appearance*/)     { return false; }
void hidDeviceStop()                             {}
bool hidDeviceIsConnected()                      { return false; }
bool hidSendKeyboard(const uint8_t[8])           { return false; }
bool hidSendConsumer(const uint8_t[2])           { return false; }
bool hidSendMouse(const uint8_t[4])              { return false; }
bool hidSendGamepad(const uint8_t[8])            { return false; }

// endregion Public functions

class BluetoothService final : public Service {

public:

    bool onStart(ServiceContext& service) override {
        check(bt == nullptr);
        bt = new Bluetooth();
        return true;
    }

    void onStop(ServiceContext& service) override {
        check(bt != nullptr);
        delete bt;
        bt = nullptr;
    }
};

extern const ServiceManifest manifest = {
    .id = "bluetooth",
    .createService = create<BluetoothService>
};

} // namespace tt::service::bluetooth

#endif // not defined(CONFIG_BT_NIMBLE_ENABLED)
