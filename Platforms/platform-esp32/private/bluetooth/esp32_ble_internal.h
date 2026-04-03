#pragma once

#ifdef ESP_PLATFORM
#include <sdkconfig.h>
#endif

#if defined(CONFIG_BT_NIMBLE_ENABLED)

#include <tactility/drivers/bluetooth.h>
#include <tactility/drivers/bluetooth_serial.h>
#include <tactility/drivers/bluetooth_midi.h>
#include <tactility/drivers/bluetooth_hid_device.h>
#include <tactility/error.h>
// Must be included before any NimBLE header: log_common.h (pulled in by ble_hs.h)
// defines LOG_LEVEL_* as macros with the same names as tactility/log.h's LogLevel enum.
// Including tactility/log.h first ensures the enum is compiled before the macros shadow it.
#include <tactility/log.h>

#include <host/ble_gap.h>
#include <host/ble_gatt.h>
#include <host/ble_hs.h>
#include <host/ble_uuid.h>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <atomic>
#include <deque>
#include <vector>

// ---- HID profile selection ----

enum class BleHidProfile { None, KbConsumer, Mouse, KbMouse, Gamepad };

// ---- BleCtx ----

#define BLE_MAX_CALLBACKS 8
#define BLE_SCAN_MAX 64

struct BleCallbackEntry {
    BtEventCallback fn;
    void* ctx;
};

struct BleCtx {
    // Mutexes
    SemaphoreHandle_t radio_mutex;  // guards radio state transitions
    SemaphoreHandle_t data_mutex;   // guards scan results + RX queues
    SemaphoreHandle_t cb_mutex;     // guards callbacks array

    // Radio / scan state (atomic — read from multiple tasks)
    std::atomic<BtRadioState> radio_state;
    std::atomic<bool> scan_active;
    // Set by Tactility HID host to prevent simultaneous central connection during name resolution
    std::atomic<bool> hid_host_active;

    // Event callbacks (guarded by cb_mutex)
    BleCallbackEntry callbacks[BLE_MAX_CALLBACKS];
    size_t callback_count;

    // Scan results (guarded by data_mutex)
    BtPeerRecord scan_results[BLE_SCAN_MAX];
    ble_addr_t   scan_addrs[BLE_SCAN_MAX]; // parallel: full ble_addr_t (type+val) for connections
    size_t       scan_count;

    // RX queues (guarded by data_mutex, capped at 16 packets each)
    std::deque<std::vector<uint8_t>> spp_rx_queue;
    std::deque<std::vector<uint8_t>> midi_rx_queue;

    // Connection handles + active flags (atomic — accessed from multiple tasks)
    std::atomic<uint16_t> spp_conn_handle;
    std::atomic<bool>     spp_active;
    std::atomic<uint16_t> midi_conn_handle;
    std::atomic<bool>     midi_active;
    std::atomic<bool>     midi_use_indicate;  // true when client subscribed for INDICATE (e.g. Windows)
    std::atomic<uint16_t> hid_conn_handle;
    std::atomic<bool>     hid_active;
    std::atomic<bool>     link_encrypted;
    std::atomic<int>      pending_reset_count;

    // Timers
    esp_timer_handle_t midi_keepalive_timer; // 2-second periodic Active Sensing
    esp_timer_handle_t adv_restart_timer;    // one-shot after connect failure (500 ms)
    // One-shot timer used to dispatch dispatchDisable off the NimBLE host task.
    // nimble_port_stop() must not be called from the NimBLE host task itself.
    esp_timer_handle_t disable_timer;

    // BLE device name (set before or after radio enable; applied in dispatch_enable)
    char device_name[BLE_DEVICE_NAME_MAX + 1];

    // Device reference (passed to BtEventCallback)
    struct Device* device;

    // Child devices (created by esp32_ble_start_device, destroyed by stop_device)
    struct Device* serial_child;
    struct Device* midi_child;
    struct Device* hid_device_child;
};

// ---- Context accessor (defined in esp32_ble.cpp) ----
// Retrieves the BleCtx stored in the device driver-data slot.
// All cross-module functions accept Device* and call this internally.
BleCtx* ble_get_ctx(struct Device* device);

// ---- Event publishing ----
void ble_publish_event(struct Device* device, struct BtEvent event);

// ---- Advertising helpers (defined in esp32_ble.cpp) ----
void ble_start_advertising(struct Device* device, const ble_uuid128_t* svc_uuid);  // svc_uuid=nullptr → name-only
void ble_start_advertising_hid(struct Device* device, uint16_t appearance);
void ble_schedule_adv_restart(struct Device* device, uint64_t delay_us);

// ---- GAP scan callback (defined in esp32_ble_scan.cpp) ----
int  ble_gap_disc_event_handler(struct ble_gap_event* event, void* arg);
void ble_resolve_next_unnamed_peer(struct Device* device, size_t start_idx);

// ---- SPP GATT (defined in esp32_ble_spp.cpp) ----
void    ble_spp_init_gatt_handles(struct Device* device);
error_t ble_spp_start_internal(struct Device* device);

// ---- MIDI GATT (defined in esp32_ble_midi.cpp) ----
void    ble_midi_init_gatt_handles(struct Device* device);
error_t ble_midi_start_internal(struct Device* device);

// ---- HID device GATT (defined in esp32_ble_hid_device.cpp) ----
void ble_hid_device_init_gatt();
void ble_hid_device_init_gatt_handles();
void ble_hid_device_switch_profile(struct Device* device, BleHidProfile profile);

// ---- Cross-module GATT char / service arrays ----
// Non-const: the .arg field is set to the parent Device* at init time so that
// NimBLE access callbacks can retrieve the context without a global pointer.
extern struct ble_gatt_chr_def nus_chars_with_handle[];  // esp32_ble_spp.cpp
extern struct ble_gatt_chr_def midi_chars[];              // esp32_ble_midi.cpp

// ---- Cross-module service UUIDs ----
extern const ble_uuid128_t NUS_SVC_UUID;   // esp32_ble_spp.cpp
extern const ble_uuid128_t MIDI_SVC_UUID;  // esp32_ble_midi.cpp

// ---- Cross-module GATT handle variables ----
extern uint16_t nus_tx_handle;              // esp32_ble_spp.cpp
extern uint16_t midi_io_handle;             // esp32_ble_midi.cpp
extern uint16_t hid_kb_input_handle;        // esp32_ble_hid_device.cpp
extern uint16_t hid_consumer_input_handle;  // esp32_ble_hid_device.cpp
extern uint16_t hid_mouse_input_handle;     // esp32_ble_hid_device.cpp
extern uint16_t hid_gamepad_input_handle;   // esp32_ble_hid_device.cpp

// ---- HID active report map / appearance ----
extern const uint8_t* active_hid_rpt_map;    // esp32_ble_hid_device.cpp
extern size_t         active_hid_rpt_map_len; // esp32_ble_hid_device.cpp
extern uint16_t       hid_appearance;         // esp32_ble_hid_device.cpp
extern BleHidProfile  current_hid_profile;    // esp32_ble_hid_device.cpp

// ---- Cross-module sub-API structs ----
extern const BtHidDeviceApi nimble_hid_device_api; // esp32_ble_hid_device.cpp
extern const BtSerialApi    nimble_serial_api;     // esp32_ble_spp.cpp
extern const BtMidiApi      nimble_midi_api;       // esp32_ble_midi.cpp

#endif // CONFIG_BT_NIMBLE_ENABLED
