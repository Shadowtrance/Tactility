#include "tt_bluetooth.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <Tactility/service/bluetooth/Bluetooth.h>

using namespace tt::service;

// Helper to convert a raw 6-byte array (which may not be pointer-safe to
// pass directly) into the std::array the C++ API expects.
static std::array<uint8_t, 6> toAddrArray(const uint8_t addr[6]) {
    std::array<uint8_t, 6> a;
    std::copy(addr, addr + 6, a.begin());
    return a;
}

extern "C" {

TtBtRadioState tt_bt_get_radio_state() {
    return static_cast<TtBtRadioState>(bluetooth::getRadioState());
}

const char* tt_bt_radio_state_to_string(TtBtRadioState state) {
    return bluetooth::radioStateToString(static_cast<bluetooth::RadioState>(state));
}

void tt_bt_set_enabled(bool enabled) {
    bluetooth::setEnabled(enabled);
}

void tt_bt_scan_start() {
    bluetooth::scanStart();
}

void tt_bt_scan_stop() {
    bluetooth::scanStop();
}

bool tt_bt_is_scanning() {
    return bluetooth::isScanning();
}

void tt_bt_connect(const uint8_t addr[6], int profile_id) {
    if (addr == nullptr) return;
    bluetooth::connect(toAddrArray(addr), profile_id);
}

void tt_bt_disconnect(const uint8_t addr[6], int profile_id) {
    if (addr == nullptr) return;
    bluetooth::disconnect(toAddrArray(addr), profile_id);
}

void tt_bt_pair(const uint8_t addr[6]) {
    if (addr == nullptr) return;
    bluetooth::pair(toAddrArray(addr));
}

void tt_bt_unpair(const uint8_t addr[6]) {
    if (addr == nullptr) return;
    bluetooth::unpair(toAddrArray(addr));
}

bool tt_bt_is_profile_supported(int profile_id) {
    return bluetooth::isProfileSupported(profile_id);
}

size_t tt_bt_get_scan_results(TtBtPeer* out, size_t max) {
    if (out == nullptr || max == 0) return 0;
    auto results = bluetooth::getScanResults();
    size_t count = std::min(results.size(), max);
    for (size_t i = 0; i < count; ++i) {
        const auto& r = results[i];
        std::copy(r.addr.begin(), r.addr.end(), out[i].addr);
        std::strncpy(out[i].name, r.name.c_str(), TT_BT_NAME_MAX);
        out[i].name[TT_BT_NAME_MAX] = '\0';
        out[i].rssi      = r.rssi;
        out[i].paired    = r.paired;
        out[i].connected = r.connected;
    }
    return count;
}

void tt_bt_hid_host_connect(const uint8_t addr[6]) {
    if (addr == nullptr) return;
    bluetooth::hidHostConnect(toAddrArray(addr));
}

void tt_bt_hid_host_disconnect() {
    bluetooth::hidHostDisconnect();
}

bool tt_bt_hid_host_is_connected() {
    return bluetooth::hidHostIsConnected();
}

bool tt_bt_hid_device_start(uint16_t appearance)     { return bluetooth::hidDeviceStart(appearance); }
void tt_bt_hid_device_stop()                         { bluetooth::hidDeviceStop(); }
bool tt_bt_hid_device_is_connected()                 { return bluetooth::hidDeviceIsConnected(); }
bool tt_bt_hid_send_keyboard(const uint8_t r[8])     { return bluetooth::hidSendKeyboard(r); }
bool tt_bt_hid_send_consumer(const uint8_t r[2])     { return bluetooth::hidSendConsumer(r); }
bool tt_bt_hid_send_mouse(const uint8_t r[4])        { return bluetooth::hidSendMouse(r); }
bool tt_bt_hid_send_gamepad(const uint8_t r[8])      { return bluetooth::hidSendGamepad(r); }

bool tt_bt_spp_start()                               { return bluetooth::sppStart(); }
void tt_bt_spp_stop()                                { bluetooth::sppStop(); }
bool tt_bt_spp_write(const uint8_t* data, size_t len){ return bluetooth::sppWrite(data, len); }
size_t tt_bt_spp_read(uint8_t* data, size_t max)     { return bluetooth::sppRead(data, max); }
bool tt_bt_spp_is_connected()                        { return bluetooth::sppIsConnected(); }

bool tt_bt_midi_start()                              { return bluetooth::midiStart(); }
void tt_bt_midi_stop()                               { bluetooth::midiStop(); }
bool tt_bt_midi_send(const uint8_t* msg, size_t len) { return bluetooth::midiSend(msg, len); }
bool tt_bt_midi_is_connected()                       { return bluetooth::midiIsConnected(); }
size_t tt_bt_midi_read(uint8_t* data, size_t max)    { return bluetooth::midiRead(data, max); }

} // extern "C"
