#pragma once

#if defined(CONFIG_BT_NIMBLE_ENABLED)

#include <Tactility/service/bluetooth/Bluetooth.h>
#include <Tactility/service/bluetooth/BluetoothPairedDevice.h>
#include <Tactility/service/bluetooth/BluetoothSettings.h>

#include <tactility/check.h>
#include <Tactility/Logger.h>
#include <Tactility/Mutex.h>
#include <Tactility/RecursiveMutex.h>
#include <Tactility/Tactility.h>
#include <Tactility/service/Service.h>
#include <Tactility/service/ServiceContext.h>
#include <Tactility/service/ServiceManifest.h>

#include <tactility/drivers/bluetooth.h>

#include <host/ble_att.h>
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <host/ble_sm.h>
#include <host/ble_store.h>
#include <host/ble_uuid.h>
#include <host/util/util.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>
#include <store/config/ble_store_config.h>

#include <esp_timer.h>

#include <atomic>
#include <cstring>
#include <deque>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <lvgl.h>
#include <Tactility/Assets.h>
#include <Tactility/lvgl/Keyboard.h>
#include <Tactility/lvgl/LvglSync.h>
#include <vector>

namespace tt::service::bluetooth {

// ---- Bluetooth singleton class ----

class Bluetooth {
public:
    RecursiveMutex radioMutex;
    RecursiveMutex dataMutex;
    std::atomic<RadioState> radioState = RadioState::Off;
    std::atomic<bool> scanActive = false;
    std::shared_ptr<PubSub<BtEvent>> pubsub = std::make_shared<PubSub<BtEvent>>();

    // Scan results (scanAddresses is a parallel vector — same index = same device).
    // scanAddresses stores the full ble_addr_t (type + value) so that name resolution
    // can connect back to the device using the correct address type (random vs public).
    std::vector<PeerRecord> scanResults;
    std::vector<ble_addr_t> scanAddresses;

    // SPP RX queue: packets received on NUS RX characteristic.
    // Protected by dataMutex. Capped at 16 packets to bound memory.
    std::deque<std::vector<uint8_t>> sppRxQueue;

    // MIDI RX queue: raw BLE MIDI packets received on the MIDI I/O characteristic.
    // Each packet includes the 2-byte BLE MIDI header. Capped at 16 packets.
    std::deque<std::vector<uint8_t>> midiRxQueue;

    // GATT handles (for SPP and MIDI server)
    uint16_t nusTxHandle = 0;
    uint16_t midiIoHandle = 0;

    // SPP connection handle + active flag.
    // sppConnHandle is written from the NimBLE host task and read from app tasks
    // (sppWrite, sppIsConnected) — must be atomic.
    std::atomic<uint16_t> sppConnHandle = BLE_HS_CONN_HANDLE_NONE;
    bool sppActive = false;

    // MIDI connection handle + active flag.
    // midiConnHandle is written from the NimBLE host task and read from app tasks
    // (midiSend, midiIsConnected, keepalive timer callback) — must be atomic.
    std::atomic<uint16_t> midiConnHandle = BLE_HS_CONN_HANDLE_NONE;
    bool midiActive = false;
    bool midiUseIndicate = false; // true when client subscribed for INDICATE (e.g. Windows)

    // Periodic Active Sensing timer — fires every 2s while MIDI is connected
    // to prevent Windows BLE MIDI's ~8-10s idle-timeout disconnect.
    esp_timer_handle_t midiKeepaliveTimer = nullptr;

    // One-shot timer that restarts advertising after a short delay.
    // Used after BLE_GAP_EVENT_CONNECT failures so NimBLE has time to clean
    // up internal state (SMP, connection table) before the peer retries.
    // Without the delay, rapid retries hit NimBLE before it's ready → EAGAIN loop.
    esp_timer_handle_t advRestartTimer = nullptr;

    // Tracks whether the current connection has established encryption.
    // Used to send MIDI Active Sensing on the encrypted channel.
    bool linkEncrypted = false;

    // HID device connection handle + active flag.
    // hidConnHandle is set from SUBSCRIBE events (NimBLE host task), read from Send functions.
    std::atomic<uint16_t> hidConnHandle = BLE_HS_CONN_HANDLE_NONE;
    bool hidActive = false;

    // Reset recovery: count resets while still in OnPending (controller unresponsive)
    std::atomic<int> pendingResetCount = 0;

    RadioState getRadioState() const { return radioState.load(); }
    void setRadioState(RadioState s) { radioState.store(s); }
    bool isScanning() const { return scanActive.load(); }
    void setScanning(bool s) { scanActive.store(s); }
};

extern std::shared_ptr<Bluetooth> bt_singleton;

// ---- HID Host context (forward declaration for cross-module use) ----
// Full definition lives in BluetoothNimBLEHidHost.cpp; other files only need
// to check hid_host_ctx != nullptr (bool-ness) or access peerAddr / connHandle.

struct HidHostInputRpt {
    uint16_t valHandle;
    uint16_t cccdHandle; // 0 = not yet discovered
    uint8_t  reportId;
};

struct HidHostCtx {
    std::shared_ptr<Bluetooth> bt;
    uint16_t connHandle       = BLE_HS_CONN_HANDLE_NONE;
    uint16_t hidSvcStart      = 0;
    uint16_t hidSvcEnd        = 0;
    std::vector<HidHostInputRpt> inputRpts;
    int subscribeIdx          = 0;
    bool securityInitiated    = false;
    bool readyBlockFired      = false; // prevents duplicate execution if timer fires after subscribe
    lv_indev_t* kbIndev       = nullptr;
    lv_indev_t* mouseIndev    = nullptr;
    lv_obj_t*   mouseCursor   = nullptr;
    std::array<uint8_t, 6> peerAddr = {}; // address of the connected peer
};

extern std::unique_ptr<HidHostCtx> hid_host_ctx; // defined in BluetoothNimBLEHidHost.cpp

// ---- HID profile selection ----

enum class HidProfile { None, KbConsumer, Mouse, KbMouse, Gamepad };
extern HidProfile current_hid_profile; // defined in BluetoothNimBLEHidDevice.cpp

// ---- Cross-module GATT handle externs ----

extern uint16_t nus_tx_handle;          // defined in BluetoothNimBLESpp.cpp
extern uint16_t midi_io_handle;         // defined in BluetoothNimBLEMidi.cpp
extern uint16_t hid_kb_input_handle;    // defined in BluetoothNimBLEHidDevice.cpp
extern uint16_t hid_consumer_input_handle;
extern uint16_t hid_mouse_input_handle;
extern uint16_t hid_gamepad_input_handle;

// ---- Cross-module GATT char array externs ----
// (needed by gatt_svcs_* arrays in HidDevice.cpp which embed NUS and MIDI chars)

extern const ble_gatt_chr_def nus_chars_with_handle[];  // defined in BluetoothNimBLESpp.cpp
extern const ble_gatt_chr_def midi_chars[];              // defined in BluetoothNimBLEMidi.cpp

// ---- Cross-module service UUID externs ----
// (needed by gatt_svcs_* arrays in HidDevice.cpp and advRestartCallback in core)

extern const ble_uuid128_t NUS_SVC_UUID;   // defined in BluetoothNimBLESpp.cpp
extern const ble_uuid128_t MIDI_SVC_UUID;  // defined in BluetoothNimBLEMidi.cpp

// ---- HID active report map (set by switchGattProfile, read by hidChrAccess and dispatchDisable) ----
extern const uint8_t* active_hid_rpt_map;    // defined in BluetoothNimBLEHidDevice.cpp
extern size_t         active_hid_rpt_map_len; // defined in BluetoothNimBLEHidDevice.cpp

// ---- HID appearance (set by hidDeviceStart, used by advRestartCallback in core) ----
extern uint16_t hid_appearance; // defined in BluetoothNimBLEHidDevice.cpp

// ---- HID enc retry timer (defined in BluetoothNimBLEHidHost.cpp, created/destroyed in service) ----
extern esp_timer_handle_t hid_enc_retry_timer;

// ---- Cross-module API struct externs ----

extern const BtHidApi    nimble_hid_api;    // defined in BluetoothNimBLEHidDevice.cpp
extern const BtSerialApi nimble_serial_api; // defined in BluetoothNimBLESpp.cpp
extern const BtMidiApi   nimble_midi_api;   // defined in BluetoothNimBLEMidi.cpp

// ---- Cross-module function declarations ----

// From BluetoothNimBLE.cpp (core)
void publishEvent(std::shared_ptr<Bluetooth> bt, BtEvent event);
void startAdvertising(const ble_uuid128_t* svcUuid = nullptr);
void startAdvertisingHid(uint16_t appearance = 0x03C1);
void scheduleAdvRestart(std::shared_ptr<Bluetooth> bt, uint64_t delay_us = 0);

// From BluetoothNimBLEScan.cpp
int gapDiscEventHandler(struct ble_gap_event* event, void* arg);
void resolveNextUnnamedPeer(std::shared_ptr<Bluetooth> bt, size_t start_idx);
void dispatchAutoConnectHidHost(std::shared_ptr<Bluetooth> bt);

// From BluetoothNimBLESpp.cpp
void sppInitGattHandles();
error_t sppStartInternal();  // calls internal Device* version without touching settings

// From BluetoothNimBLEMidi.cpp
void midiInitGattHandles();
error_t midiStartInternal();  // calls internal Device* version without touching settings

// From BluetoothNimBLEHidDevice.cpp
void hidDeviceInitGatt();
void hidDeviceInitGattHandles();
void switchGattProfile(HidProfile profile);

// From BluetoothNimBLEHidHost.cpp
int hidHostGapCb(struct ble_gap_event* event, void* arg);
void hidHostConnect(const std::array<uint8_t, 6>& addr);
void hidHostDisconnect();
bool hidHostIsConnectedImpl();
void hidEncRetryTimerCb(void* arg);

} // namespace tt::service::bluetooth

#endif // CONFIG_BT_NIMBLE_ENABLED
