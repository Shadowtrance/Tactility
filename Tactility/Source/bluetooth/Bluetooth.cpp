#ifdef ESP_PLATFORM
#include <sdkconfig.h>
#endif

#if defined(CONFIG_BT_NIMBLE_ENABLED)

#include <Tactility/bluetooth/Bluetooth.h>
#include <Tactility/bluetooth/BluetoothPairedDevice.h>
#include <Tactility/bluetooth/BluetoothSettings.h>
#include <Tactility/bluetooth/BluetoothPrivate.h>

#include <Tactility/Logger.h>
#include <Tactility/Mutex.h>
#include <Tactility/PubSub.h>
#include <Tactility/Tactility.h>
#include <tactility/check.h>
#include <tactility/device.h>
#include <tactility/drivers/bluetooth.h>

#include <array>
#include <cstring>
#include <vector>

namespace tt::bluetooth {

static const auto LOGGER = Logger("Bluetooth");

// ---- Shared pubsub ----

static std::shared_ptr<PubSub<BtEvent>> bt_pubsub = std::make_shared<PubSub<BtEvent>>();

// ---- Scan result cache (C++ PeerRecord list, updated from BT_EVENT_PEER_FOUND) ----

static Mutex scan_cache_mutex;
static std::vector<PeerRecord> scan_results_cache;

struct CachedAddr {
    uint8_t addr[6];
    uint8_t addr_type;
};
static std::vector<CachedAddr> scan_addr_cache; // parallel to scan_results_cache

// ---- Device / API accessors ----

struct Device* getDevice() {
    struct Device* found = nullptr;
    device_for_each_of_type(&BLUETOOTH_TYPE, &found, [](struct Device* dev, void* ctx) -> bool {
        if (device_is_ready(dev)) {
            *static_cast<struct Device**>(ctx) = dev;
            return false;
        }
        return true;
    });
    return found;
}

// ---- Event publishing ----

void publishEventCpp(BtEvent event) {
    bt_pubsub->publish(event);
}

// ---- Scan cache helpers ----

void cacheScanAddr(const uint8_t addr[6], uint8_t addr_type) {
    auto lock = scan_cache_mutex.asScopedLock();
    lock.lock();
    for (auto& entry : scan_addr_cache) {
        if (memcmp(entry.addr, addr, 6) == 0) {
            entry.addr_type = addr_type;
            return;
        }
    }
    CachedAddr e = {};
    memcpy(e.addr, addr, 6);
    e.addr_type = addr_type;
    scan_addr_cache.push_back(e);
}

bool getCachedScanAddrType(const uint8_t addr[6], uint8_t* addr_type_out) {
    auto lock = scan_cache_mutex.asScopedLock();
    lock.lock();
    for (const auto& entry : scan_addr_cache) {
        if (memcmp(entry.addr, addr, 6) == 0) {
            if (addr_type_out) *addr_type_out = entry.addr_type;
            return true;
        }
    }
    if (addr_type_out) *addr_type_out = 0;
    return false;
}

static void cachePeerRecord(const BtPeerRecord& krecord) {
    PeerRecord rec;
    memcpy(rec.addr.data(), krecord.addr, 6);
    rec.name      = krecord.name[0] != '\0' ? krecord.name : "";
    rec.rssi      = krecord.rssi;
    rec.paired    = krecord.paired;
    rec.connected = krecord.connected;
    rec.profileId = 0;

    cacheScanAddr(krecord.addr, krecord.addr_type);

    auto lock = scan_cache_mutex.asScopedLock();
    lock.lock();
    for (auto& existing : scan_results_cache) {
        if (existing.addr == rec.addr) {
            if (!rec.name.empty()) existing.name = rec.name;
            existing.rssi = rec.rssi;
            return;
        }
    }
    scan_results_cache.push_back(std::move(rec));
}

// ---- Auto-connect HID host after scan ----

void dispatchAutoConnectHidHost() {
    auto peers = settings::loadAll();
    for (const auto& peer : peers) {
        if (peer.autoConnect && peer.profileId == BT_PROFILE_HID_HOST) {
            LOGGER.info("Auto-connecting HID host to {}", settings::addrToHex(peer.addr));
            hidHostConnect(peer.addr);
            break;
        }
    }
}

// ---- Bridge callback (registered with kernel driver) ----

static void bt_event_bridge(struct Device* /*device*/, void* /*context*/, struct BtEvent event) {
    switch (event.type) {
        case BT_EVENT_RADIO_STATE_CHANGED:
            switch (event.radio_state) {
                case BT_RADIO_STATE_ON:
                    publishEventCpp(BtEvent::RadioStateOn);
                    getMainDispatcher().dispatch([] {
                        if (settings::shouldSppAutoStart()) {
                            LOGGER.info("Auto-starting SPP server");
                            sppStart();
                        } else if (settings::shouldMidiAutoStart()) {
                            LOGGER.info("Auto-starting MIDI server");
                            midiStart();
                        }
                    });
                    break;
                case BT_RADIO_STATE_OFF:
                    publishEventCpp(BtEvent::RadioStateOff);
                    break;
                case BT_RADIO_STATE_ON_PENDING:
                    publishEventCpp(BtEvent::RadioStateOnPending);
                    break;
                case BT_RADIO_STATE_OFF_PENDING:
                    publishEventCpp(BtEvent::RadioStateOffPending);
                    break;
            }
            break;

        case BT_EVENT_SCAN_STARTED:
            {
                auto lock = scan_cache_mutex.asScopedLock();
                lock.lock();
                scan_results_cache.clear();
                scan_addr_cache.clear();
            }
            publishEventCpp(BtEvent::ScanStarted);
            break;

        case BT_EVENT_SCAN_FINISHED:
            publishEventCpp(BtEvent::ScanFinished);
            getMainDispatcher().dispatch([] { dispatchAutoConnectHidHost(); });
            break;

        case BT_EVENT_PEER_FOUND:
            cachePeerRecord(event.peer);
            publishEventCpp(BtEvent::PeerFound);
            break;

        case BT_EVENT_PAIR_REQUEST:
            publishEventCpp(BtEvent::PairRequest);
            break;

        case BT_EVENT_PAIR_RESULT:
            if (event.pair_result.result == BT_PAIR_RESULT_SUCCESS) {
                uint8_t addr_buf[6];
                int profile_copy = event.pair_result.profile;
                memcpy(addr_buf, event.pair_result.addr, 6);
                getMainDispatcher().dispatch([addr_buf, profile_copy]() mutable {
                    std::array<uint8_t, 6> peer_addr;
                    memcpy(peer_addr.data(), addr_buf, 6);
                    const auto hex = settings::addrToHex(peer_addr);
                    if (!settings::contains(hex)) {
                        settings::PairedDevice dev;
                        dev.addr        = peer_addr;
                        dev.name        = "";
                        dev.autoConnect = true;
                        dev.profileId   = profile_copy;
                        if (settings::save(dev)) {
                            LOGGER.info("Saved paired peer {} (profile={})", hex, profile_copy);
                            publishEventCpp(BtEvent::PairSuccess);
                        }
                    }
                });
            } else if (event.pair_result.result == BT_PAIR_RESULT_BOND_LOST) {
                uint8_t addr_buf[6];
                memcpy(addr_buf, event.pair_result.addr, 6);
                getMainDispatcher().dispatch([addr_buf]() mutable {
                    std::array<uint8_t, 6> peer_addr;
                    memcpy(peer_addr.data(), addr_buf, 6);
                    settings::remove(settings::addrToHex(peer_addr));
                });
            } else {
                publishEventCpp(BtEvent::PairFailed);
            }
            break;

        case BT_EVENT_PROFILE_STATE_CHANGED:
            if (event.profile_state.state == BT_PROFILE_STATE_CONNECTED) {
                uint8_t addr_buf[6];
                int profile_copy = (int)event.profile_state.profile;
                memcpy(addr_buf, event.profile_state.addr, 6);
                getMainDispatcher().dispatch([addr_buf, profile_copy]() mutable {
                    std::array<uint8_t, 6> peer_addr;
                    memcpy(peer_addr.data(), addr_buf, 6);
                    const auto hex = settings::addrToHex(peer_addr);
                    settings::PairedDevice stored;
                    if (settings::load(hex, stored) && stored.profileId != profile_copy) {
                        stored.profileId = profile_copy;
                        settings::save(stored);
                    }
                    publishEventCpp(BtEvent::ProfileStateChanged);
                });
            } else {
                publishEventCpp(BtEvent::ProfileStateChanged);
            }
            break;

        case BT_EVENT_SPP_DATA_RECEIVED:
            publishEventCpp(BtEvent::SppDataReceived);
            break;

        case BT_EVENT_MIDI_DATA_RECEIVED:
            publishEventCpp(BtEvent::MidiDataReceived);
            break;

        case BT_EVENT_CONNECT_STATE_CHANGED:
            break;
    }
}

// ---- systemStart ----

void systemStart() {
    struct Device* dev = getDevice();
    if (dev == nullptr) {
        LOGGER.warn("systemStart: no BLE device found");
        return;
    }
    bluetooth_add_event_callback(dev, nullptr, bt_event_bridge);

    if (settings::shouldEnableOnBoot()) {
        LOGGER.info("Auto-enabling Bluetooth on boot");
        bluetooth_set_radio_enabled(dev, true);
    }
}

// ---- Public API ----

std::shared_ptr<PubSub<BtEvent>> getPubsub() {
    return bt_pubsub;
}

const char* radioStateToString(RadioState state) {
    switch (state) {
        using enum RadioState;
        case Off:        return "Off";
        case OnPending:  return "OnPending";
        case On:         return "On";
        case OffPending: return "OffPending";
    }
    check(false, "not implemented");
}

RadioState getRadioState() {
    struct Device* dev = getDevice();
    if (dev == nullptr) return RadioState::Off;
    BtRadioState state = BT_RADIO_STATE_OFF;
    bluetooth_get_radio_state(dev, &state);
    switch (state) {
        case BT_RADIO_STATE_OFF:         return RadioState::Off;
        case BT_RADIO_STATE_ON_PENDING:  return RadioState::OnPending;
        case BT_RADIO_STATE_ON:          return RadioState::On;
        case BT_RADIO_STATE_OFF_PENDING: return RadioState::OffPending;
    }
    return RadioState::Off;
}

void setEnabled(bool enabled) {
    struct Device* dev = getDevice();
    if (dev == nullptr) return;
    bluetooth_set_radio_enabled(dev, enabled);
}

void scanStart() {
    struct Device* dev = getDevice();
    if (dev == nullptr) return;
    bluetooth_scan_start(dev);
}

void scanStop() {
    struct Device* dev = getDevice();
    if (dev == nullptr) return;
    bluetooth_scan_stop(dev);
}

bool isScanning() {
    struct Device* dev = getDevice();
    if (dev == nullptr) return false;
    return bluetooth_is_scanning(dev);
}

std::vector<PeerRecord> getScanResults() {
    auto lock = scan_cache_mutex.asScopedLock();
    lock.lock();
    return scan_results_cache;
}

std::vector<PeerRecord> getPairedPeers() {
    auto stored = settings::loadAll();
    std::vector<PeerRecord> result;
    result.reserve(stored.size());
    bool hid_host_connected = hidHostIsConnected();
    for (const auto& device : stored) {
        PeerRecord record;
        record.addr      = device.addr;
        record.name      = device.name;
        record.rssi      = 0;
        record.paired    = true;
        record.profileId = device.profileId;
        record.connected = hid_host_connected && device.profileId == BT_PROFILE_HID_HOST;
        result.push_back(std::move(record));
    }
    return result;
}

void pair(const std::array<uint8_t, 6>& /*addr*/) {
    // Pairing is handled automatically during connection by NimBLE SM.
}

void unpair(const std::array<uint8_t, 6>& addr) {
    struct Device* dev = getDevice();
    if (dev != nullptr) {
        bluetooth_unpair(dev, addr.data());
    }
    settings::remove(settings::addrToHex(addr));
}

void connect(const std::array<uint8_t, 6>& addr, int profileId) {
    LOGGER.info("connect(profile={})", profileId);
    if (profileId == BT_PROFILE_HID_HOST) {
        hidHostConnect(addr);
    } else if (profileId == BT_PROFILE_HID_DEVICE) {
        hidDeviceStart();
    } else if (profileId == BT_PROFILE_SPP) {
        sppStart();
    } else if (profileId == BT_PROFILE_MIDI) {
        midiStart();
    }
}

void disconnect(const std::array<uint8_t, 6>& addr, int profileId) {
    LOGGER.info("disconnect(profile={})", profileId);
    if (profileId == BT_PROFILE_HID_HOST) {
        hidHostDisconnect();
    } else if (profileId == BT_PROFILE_HID_DEVICE) {
        hidDeviceStop();
    } else {
        struct Device* dev = getDevice();
        if (dev == nullptr) return;
        bluetooth_disconnect(dev, addr.data(), (BtProfileId)profileId);
    }
}

bool isProfileSupported(int profileId) {
    return profileId == BT_PROFILE_HID_HOST ||
           profileId == BT_PROFILE_HID_DEVICE ||
           profileId == BT_PROFILE_SPP ||
           profileId == BT_PROFILE_MIDI;
}

bool sppStart() {
    struct Device* dev = getDevice();
    if (dev == nullptr) return false;
    settings::setSppAutoStart(true);
    return bluetooth_serial_start(dev) == ERROR_NONE;
}

void sppStop() {
    struct Device* dev = getDevice();
    if (dev == nullptr) return;
    settings::setSppAutoStart(false);
    bluetooth_serial_stop(dev);
}

bool sppWrite(const uint8_t* data, size_t len) {
    struct Device* dev = getDevice();
    if (dev == nullptr) return false;
    size_t written = 0;
    return bluetooth_serial_write(dev, data, len, &written) == ERROR_NONE;
}

size_t sppRead(uint8_t* data, size_t max_len) {
    struct Device* dev = getDevice();
    if (dev == nullptr) return 0;
    size_t read_out = 0;
    bluetooth_serial_read(dev, data, max_len, &read_out);
    return read_out;
}

bool sppIsConnected() {
    struct Device* dev = getDevice();
    if (dev == nullptr) return false;
    return bluetooth_serial_is_connected(dev);
}

bool midiStart() {
    struct Device* dev = getDevice();
    if (dev == nullptr) return false;
    settings::setMidiAutoStart(true);
    return bluetooth_midi_start(dev) == ERROR_NONE;
}

void midiStop() {
    struct Device* dev = getDevice();
    if (dev == nullptr) return;
    settings::setMidiAutoStart(false);
    bluetooth_midi_stop(dev);
}

bool midiSend(const uint8_t* msg, size_t len) {
    struct Device* dev = getDevice();
    if (dev == nullptr) return false;
    return bluetooth_midi_send(dev, msg, len) == ERROR_NONE;
}

bool midiIsConnected() {
    struct Device* dev = getDevice();
    if (dev == nullptr) return false;
    return bluetooth_midi_is_connected(dev);
}

size_t midiRead(uint8_t* /*data*/, size_t /*max_len*/) {
    // BtMidiApi has no read() function; MIDI RX is delivered via BT_EVENT_MIDI_DATA_RECEIVED.
    return 0;
}

bool hidDeviceStart(uint16_t /*appearance*/) {
    struct Device* dev = getDevice();
    if (dev == nullptr) return false;
    return bluetooth_hid_device_start(dev) == ERROR_NONE;
}

void hidDeviceStop() {
    struct Device* dev = getDevice();
    if (dev == nullptr) return;
    bluetooth_hid_device_stop(dev);
}

bool hidDeviceIsConnected() {
    return false; // tracked via BT_EVENT_PROFILE_STATE_CHANGED
}

bool hidSendKeyboard(const uint8_t /*report*/[8])  { return false; }
bool hidSendConsumer(const uint8_t /*report*/[2])  { return false; }
bool hidSendMouse(const uint8_t /*report*/[4])     { return false; }
bool hidSendGamepad(const uint8_t /*report*/[8])   { return false; }

} // namespace tt::bluetooth

#endif // CONFIG_BT_NIMBLE_ENABLED
