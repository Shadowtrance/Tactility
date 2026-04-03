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

// ---- Child device driver data structs ----
// Each struct is stored as driver data on the corresponding child Device*.

struct BleHidDeviceCtx {
    std::atomic<uint16_t> hid_conn_handle;
};

struct BleMidiCtx {
    SemaphoreHandle_t             rx_mutex;
    std::deque<std::vector<uint8_t>> rx_queue;
};

struct BleSppCtx {
    SemaphoreHandle_t             rx_mutex;
    std::deque<std::vector<uint8_t>> rx_queue;
};

// ---- BleCtx field accessors (defined in esp32_ble.cpp) ----
// Work correctly when called with either the parent BLE device or any child device.

BtRadioState ble_ctx_get_radio_state(struct Device* device);

bool     ble_ctx_get_hid_active(struct Device* device);
void     ble_ctx_set_hid_active(struct Device* device, bool v);

bool     ble_ctx_get_spp_active(struct Device* device);
void     ble_ctx_set_spp_active(struct Device* device, bool v);
uint16_t ble_ctx_get_spp_conn_handle(struct Device* device);
void     ble_ctx_set_spp_conn_handle(struct Device* device, uint16_t h);

bool     ble_ctx_get_midi_active(struct Device* device);
void     ble_ctx_set_midi_active(struct Device* device, bool v);
uint16_t ble_ctx_get_midi_conn_handle(struct Device* device);
void     ble_ctx_set_midi_conn_handle(struct Device* device, uint16_t h);
bool     ble_ctx_get_midi_use_indicate(struct Device* device);
void     ble_ctx_set_midi_use_indicate(struct Device* device, bool v);

bool     ble_ctx_get_hid_host_active(struct Device* device);
bool     ble_ctx_get_scan_active(struct Device* device);
void     ble_ctx_set_scan_active(struct Device* device, bool v);

// MIDI keepalive timer helpers — timer handle lives in BleCtx.
// ble_ctx_ensure_midi_keepalive creates the timer if needed and starts it periodically.
// ble_ctx_stop_midi_keepalive stops (but does not delete) the timer.
error_t ble_ctx_ensure_midi_keepalive(struct Device* device, esp_timer_cb_t cb, uint64_t period_us);
void    ble_ctx_stop_midi_keepalive(struct Device* device);

// ---- Scan data management (defined in esp32_ble_scan.cpp) ----
void ble_scan_init();
void ble_scan_deinit();
void ble_scan_clear_results();

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
// device must be the serial child Device*.
void    ble_spp_init_gatt_handles(struct Device* serial_child);
error_t ble_spp_start_internal(struct Device* serial_child);

// ---- MIDI GATT (defined in esp32_ble_midi.cpp) ----
// device must be the midi child Device*.
void    ble_midi_init_gatt_handles(struct Device* midi_child);
error_t ble_midi_start_internal(struct Device* midi_child);

// ---- HID device GATT (defined in esp32_ble_hid_device.cpp) ----
void ble_hid_device_init_gatt();
void ble_hid_device_init_gatt_handles();
// device must be the hid_device child Device*.
void ble_hid_device_switch_profile(struct Device* hid_child, BleHidProfile profile);

// ---- Cross-module GATT char / service arrays ----
// Non-const: the .arg field is set to the child Device* at init time so that
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
