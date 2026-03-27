#ifdef ESP_PLATFORM
#include <sdkconfig.h>
#endif

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

// ble_store_config_init() is not declared in the public header in this IDF version
// but exists in the store/config library — forward-declare it here.
extern "C" void ble_store_config_init(void);

namespace tt::service::bluetooth {

static const auto LOGGER = Logger("BtService");

// ---- NUS (Nordic UART Service) UUIDs ----

// 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
static const ble_uuid128_t NUS_SVC_UUID = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
);

// 6E400002 RX char (write from client → device)
static const ble_uuid128_t NUS_RX_UUID = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E
);

// 6E400003 TX char (notify device → client)
static const ble_uuid128_t NUS_TX_UUID = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E
);

// ---- BLE MIDI UUIDs ----

// 03B80E5A-EDE8-4B33-A751-6CE34EC4C700
static const ble_uuid128_t MIDI_SVC_UUID = BLE_UUID128_INIT(
    0x00, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C, 0x51, 0xA7,
    0x33, 0x4B, 0xE8, 0xED, 0x5A, 0x0E, 0xB8, 0x03
);

// 7772E5DB-3868-4112-A1A9-F2669D106BF3
static const ble_uuid128_t MIDI_IO_UUID = BLE_UUID128_INIT(
    0xF3, 0x6B, 0x10, 0x9D, 0x66, 0xF2, 0xA9, 0xA1,
    0x12, 0x41, 0x68, 0x38, 0xDB, 0xE5, 0x72, 0x77
);

// ---- Bluetooth singleton ----

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

static std::shared_ptr<Bluetooth> bt_singleton;

// ---- Forward declarations ----

static void publishEvent(std::shared_ptr<Bluetooth> bt, BtEvent event);
static void startAdvertising(const ble_uuid128_t* svcUuid = nullptr);
static void scheduleAdvRestart(std::shared_ptr<Bluetooth> bt, uint64_t delay_us = 0);
static void bleHostTask(void* param);
static void onSync();
static void onReset(int reason);
static void dispatchDisable(std::shared_ptr<Bluetooth> bt);
static int gapEventHandler(struct ble_gap_event* event, void* arg);
static int nusChrAccess(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt* ctxt, void* arg);
static error_t sppStart(struct Device* device);
static error_t midiStart(struct Device* device);
static int midiChrAccess(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt* ctxt, void* arg);
static void resolveNextUnnamedPeer(std::shared_ptr<Bluetooth> bt, size_t start_idx);
static int hidChrAccess(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt* ctxt, void* arg);
static int hidDscAccess(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt* ctxt, void* arg);
static void startAdvertisingHid(uint16_t appearance = 0x03C1);

// ---- HID profile selection ----

enum class HidProfile { None, KbConsumer, Mouse, KbMouse, Gamepad };
static HidProfile current_hid_profile = HidProfile::None;
static void switchGattProfile(HidProfile profile);

// ---- HID Host ----

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

static std::unique_ptr<HidHostCtx> hid_host_ctx;
static QueueHandle_t hid_host_key_queue = nullptr;
static uint8_t hid_host_prev_keys[6] = {}; // previous keyboard report keycodes for press/release diff
// One-shot timer to delay CCCD writes after ENC_CHANGE (bonded devices need time to settle)
static esp_timer_handle_t hid_enc_retry_timer = nullptr;

// Mouse state — written from NimBLE task, read from LVGL task (atomic)
static std::atomic<int32_t> hid_host_mouse_x{0};
static std::atomic<int32_t> hid_host_mouse_y{0};
static std::atomic<bool>    hid_host_mouse_btn{false};
static std::atomic<bool>    hid_host_mouse_active{false}; // set on first movement; shows cursor

#define HID_HOST_KEY_QUEUE_SIZE 64
struct HidHostKeyEvt { uint32_t key; bool pressed; };

static int hidHostGapCb(struct ble_gap_event* event, void* arg);
void hidHostConnect(const std::array<uint8_t, 6>& addr);
void hidHostDisconnect();
static bool hidHostIsConnectedImpl();

// appearance stored so advRestartCallback can re-use it without an extra parameter
static uint16_t hid_appearance = 0x03C1;

// ---- Static UUID objects for HID 16-bit UUIDs ----
// BLE_UUID16_DECLARE() takes the address of a temporary (rvalue) which is
// illegal in C++.  Declare named variables and use &var.u instead.
static const ble_uuid16_t UUID16_RPT_REF    = BLE_UUID16_INIT(0x2908);
static const ble_uuid16_t UUID16_HID_INFO   = BLE_UUID16_INIT(0x2A4A);
static const ble_uuid16_t UUID16_RPT_MAP    = BLE_UUID16_INIT(0x2A4B);
static const ble_uuid16_t UUID16_HID_CTRL   = BLE_UUID16_INIT(0x2A4C);
static const ble_uuid16_t UUID16_HID_REPORT = BLE_UUID16_INIT(0x2A4D);
static const ble_uuid16_t UUID16_PROTO_MODE = BLE_UUID16_INIT(0x2A4E);
static const ble_uuid16_t UUID16_HID_SVC    = BLE_UUID16_INIT(0x1812);

// ---- GATT service definitions ----

static uint16_t nus_tx_handle;
static uint16_t midi_io_handle;

static uint8_t hid_protocol_mode = 0x01; // 0x00=Boot, 0x01=Report (default)

// ============================================================================
// Per-profile HID Report Maps
// ============================================================================

// Keyboard + Consumer (IDs 1 and 2)
static const uint8_t hid_rpt_map_kb_consumer[] = {
    // Report 1: Keyboard — 8 bytes [modifier][reserved][key0..key5]
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID 1
    0x05, 0x07,        //   Usage Page (Key Codes)
    0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
    0x75, 0x08, 0x95, 0x01, 0x81, 0x01,
    0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x75, 0x01, 0x95, 0x05, 0x91, 0x02,
    0x75, 0x03, 0x95, 0x01, 0x91, 0x01,
    0x15, 0x00, 0x25, 0x73, 0x05, 0x07, 0x19, 0x00, 0x29, 0x73,
    0x75, 0x08, 0x95, 0x06, 0x81, 0x00,
    0xC0,
    // Report 2: Consumer / Media — 2 bytes [usage_lo][usage_hi]
    0x05, 0x0C,        // Usage Page (Consumer Devices)
    0x09, 0x01,        // Usage (Consumer Control)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x02,        //   Report ID 2
    0x15, 0x00, 0x26, 0xFF, 0x03, 0x19, 0x00, 0x2A, 0xFF, 0x03,
    0x75, 0x10, 0x95, 0x01, 0x81, 0x00,
    0xC0,
};

// Mouse only (ID 1, 4 bytes)
static const uint8_t hid_rpt_map_mouse[] = {
    0x05, 0x01, 0x09, 0x02, 0xA1, 0x01,
    0x85, 0x01,        // Report ID 1
    0x09, 0x01, 0xA1, 0x00,
    0x05, 0x09, 0x19, 0x01, 0x29, 0x05,
    0x15, 0x00, 0x25, 0x01, 0x95, 0x05, 0x75, 0x01, 0x81, 0x02,
    0x95, 0x01, 0x75, 0x03, 0x81, 0x01,
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,
    0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x03, 0x81, 0x06,
    0xC0, 0xC0,
};

// Keyboard + Consumer + Mouse (IDs 1, 2, 3)
static const uint8_t hid_rpt_map_kb_mouse[] = {
    // Report 1: Keyboard
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01,
    0x85, 0x01,
    0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
    0x75, 0x08, 0x95, 0x01, 0x81, 0x01,
    0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x75, 0x01, 0x95, 0x05, 0x91, 0x02,
    0x75, 0x03, 0x95, 0x01, 0x91, 0x01,
    0x15, 0x00, 0x25, 0x73, 0x05, 0x07, 0x19, 0x00, 0x29, 0x73,
    0x75, 0x08, 0x95, 0x06, 0x81, 0x00,
    0xC0,
    // Report 2: Consumer
    0x05, 0x0C, 0x09, 0x01, 0xA1, 0x01,
    0x85, 0x02,
    0x15, 0x00, 0x26, 0xFF, 0x03, 0x19, 0x00, 0x2A, 0xFF, 0x03,
    0x75, 0x10, 0x95, 0x01, 0x81, 0x00,
    0xC0,
    // Report 3: Mouse
    0x05, 0x01, 0x09, 0x02, 0xA1, 0x01,
    0x85, 0x03,
    0x09, 0x01, 0xA1, 0x00,
    0x05, 0x09, 0x19, 0x01, 0x29, 0x05,
    0x15, 0x00, 0x25, 0x01, 0x95, 0x05, 0x75, 0x01, 0x81, 0x02,
    0x95, 0x01, 0x75, 0x03, 0x81, 0x01,
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,
    0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x03, 0x81, 0x06,
    0xC0, 0xC0,
};

// Gamepad only (ID 1, 8 bytes)
static const uint8_t hid_rpt_map_gamepad[] = {
    0x05, 0x01, 0x09, 0x05, 0xA1, 0x01,
    0x85, 0x01,        // Report ID 1
    0x05, 0x09, 0x19, 0x01, 0x29, 0x10,
    0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x10, 0x81, 0x02,
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x09, 0x35, 0x09, 0x33, 0x09, 0x34,
    0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x06, 0x81, 0x02,
    0xC0,
};

// Active report map — updated by switchGattProfile()
static const uint8_t* active_hid_rpt_map    = nullptr;
static size_t         active_hid_rpt_map_len = 0;

// ---- Per-profile Report Reference descriptor data ----
// Format: {Report ID, Report Type} — Type: 1=Input, 2=Output

static const uint8_t rpt_ref_kbc_kb_in[2]  = {1, 1}; // KbConsumer: KB input, ID=1
static const uint8_t rpt_ref_kbc_cs_in[2]  = {2, 1}; // KbConsumer: Consumer input, ID=2
static const uint8_t rpt_ref_kbc_kb_out[2] = {1, 2}; // KbConsumer: KB LED output, ID=1

static const uint8_t rpt_ref_ms_in[2]      = {1, 1}; // Mouse: Mouse input, ID=1

static const uint8_t rpt_ref_kbm_kb_in[2]  = {1, 1}; // KbMouse: KB input, ID=1
static const uint8_t rpt_ref_kbm_cs_in[2]  = {2, 1}; // KbMouse: Consumer input, ID=2
static const uint8_t rpt_ref_kbm_ms_in[2]  = {3, 1}; // KbMouse: Mouse input, ID=3
static const uint8_t rpt_ref_kbm_kb_out[2] = {1, 2}; // KbMouse: KB LED output, ID=1

static const uint8_t rpt_ref_gp_in[2]      = {1, 1}; // Gamepad: Gamepad input, ID=1

// GATT attribute handles (updated by ble_gatts_add_svcs for the active profile)
static uint16_t hid_kb_input_handle;
static uint16_t hid_consumer_input_handle;
static uint16_t hid_mouse_input_handle;
static uint16_t hid_gamepad_input_handle;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

static const struct ble_gatt_chr_def nus_chars_with_handle[] = {
    {
        .uuid = &NUS_RX_UUID.u,
        .access_cb = nusChrAccess,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    {
        .uuid = &NUS_TX_UUID.u,
        .access_cb = nusChrAccess,
        .flags = BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &nus_tx_handle,
    },
    { 0 }
};

static const struct ble_gatt_chr_def midi_chars[] = {
    {
        .uuid = &MIDI_IO_UUID.u,
        .access_cb = midiChrAccess,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
        .val_handle = &midi_io_handle,
    },
    { 0 }
};

// ---- Per-profile HID descriptor arrays ----
// Non-const: ble_gatt_chr_def.descriptors requires a non-const pointer.

static struct ble_gatt_dsc_def hid_kbc_kb_dscs[]  = { { .uuid = &UUID16_RPT_REF.u, .att_flags = BLE_ATT_F_READ, .access_cb = hidDscAccess, .arg = (void*)rpt_ref_kbc_kb_in  }, { 0 } };
static struct ble_gatt_dsc_def hid_kbc_cs_dscs[]  = { { .uuid = &UUID16_RPT_REF.u, .att_flags = BLE_ATT_F_READ, .access_cb = hidDscAccess, .arg = (void*)rpt_ref_kbc_cs_in  }, { 0 } };
static struct ble_gatt_dsc_def hid_kbc_out_dscs[] = { { .uuid = &UUID16_RPT_REF.u, .att_flags = BLE_ATT_F_READ, .access_cb = hidDscAccess, .arg = (void*)rpt_ref_kbc_kb_out }, { 0 } };

static struct ble_gatt_dsc_def hid_ms_dscs[]      = { { .uuid = &UUID16_RPT_REF.u, .att_flags = BLE_ATT_F_READ, .access_cb = hidDscAccess, .arg = (void*)rpt_ref_ms_in      }, { 0 } };

static struct ble_gatt_dsc_def hid_kbm_kb_dscs[]  = { { .uuid = &UUID16_RPT_REF.u, .att_flags = BLE_ATT_F_READ, .access_cb = hidDscAccess, .arg = (void*)rpt_ref_kbm_kb_in  }, { 0 } };
static struct ble_gatt_dsc_def hid_kbm_cs_dscs[]  = { { .uuid = &UUID16_RPT_REF.u, .att_flags = BLE_ATT_F_READ, .access_cb = hidDscAccess, .arg = (void*)rpt_ref_kbm_cs_in  }, { 0 } };
static struct ble_gatt_dsc_def hid_kbm_ms_dscs[]  = { { .uuid = &UUID16_RPT_REF.u, .att_flags = BLE_ATT_F_READ, .access_cb = hidDscAccess, .arg = (void*)rpt_ref_kbm_ms_in  }, { 0 } };
static struct ble_gatt_dsc_def hid_kbm_out_dscs[] = { { .uuid = &UUID16_RPT_REF.u, .att_flags = BLE_ATT_F_READ, .access_cb = hidDscAccess, .arg = (void*)rpt_ref_kbm_kb_out }, { 0 } };

static struct ble_gatt_dsc_def hid_gp_dscs[]      = { { .uuid = &UUID16_RPT_REF.u, .att_flags = BLE_ATT_F_READ, .access_cb = hidDscAccess, .arg = (void*)rpt_ref_gp_in      }, { 0 } };

// ---- Per-profile HID characteristic arrays ----
// Field order: uuid → access_cb → arg → descriptors → flags → min_key_size → val_handle

// KbConsumer: Keyboard (ID 1) + Consumer (ID 2) + LED output
static struct ble_gatt_chr_def hid_chars_kb_consumer[] = {
    { .uuid = &UUID16_HID_INFO.u,   .access_cb = hidChrAccess, .flags = BLE_GATT_CHR_F_READ },
    { .uuid = &UUID16_RPT_MAP.u,    .access_cb = hidChrAccess, .flags = BLE_GATT_CHR_F_READ },
    { .uuid = &UUID16_HID_CTRL.u,   .access_cb = hidChrAccess, .flags = BLE_GATT_CHR_F_WRITE_NO_RSP },
    { .uuid = &UUID16_PROTO_MODE.u, .access_cb = hidChrAccess, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP },
    { .uuid = &UUID16_HID_REPORT.u, .access_cb = hidChrAccess, .descriptors = hid_kbc_kb_dscs,
      .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &hid_kb_input_handle },
    { .uuid = &UUID16_HID_REPORT.u, .access_cb = hidChrAccess, .descriptors = hid_kbc_cs_dscs,
      .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &hid_consumer_input_handle },
    { .uuid = &UUID16_HID_REPORT.u, .access_cb = hidChrAccess, .descriptors = hid_kbc_out_dscs,
      .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP },
    { 0 }
};

// Mouse: Mouse (ID 1)
static struct ble_gatt_chr_def hid_chars_mouse[] = {
    { .uuid = &UUID16_HID_INFO.u,   .access_cb = hidChrAccess, .flags = BLE_GATT_CHR_F_READ },
    { .uuid = &UUID16_RPT_MAP.u,    .access_cb = hidChrAccess, .flags = BLE_GATT_CHR_F_READ },
    { .uuid = &UUID16_HID_CTRL.u,   .access_cb = hidChrAccess, .flags = BLE_GATT_CHR_F_WRITE_NO_RSP },
    { .uuid = &UUID16_PROTO_MODE.u, .access_cb = hidChrAccess, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP },
    { .uuid = &UUID16_HID_REPORT.u, .access_cb = hidChrAccess, .descriptors = hid_ms_dscs,
      .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &hid_mouse_input_handle },
    { 0 }
};

// KbMouse: Keyboard (ID 1) + Consumer (ID 2) + Mouse (ID 3) + LED output
static struct ble_gatt_chr_def hid_chars_kb_mouse[] = {
    { .uuid = &UUID16_HID_INFO.u,   .access_cb = hidChrAccess, .flags = BLE_GATT_CHR_F_READ },
    { .uuid = &UUID16_RPT_MAP.u,    .access_cb = hidChrAccess, .flags = BLE_GATT_CHR_F_READ },
    { .uuid = &UUID16_HID_CTRL.u,   .access_cb = hidChrAccess, .flags = BLE_GATT_CHR_F_WRITE_NO_RSP },
    { .uuid = &UUID16_PROTO_MODE.u, .access_cb = hidChrAccess, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP },
    { .uuid = &UUID16_HID_REPORT.u, .access_cb = hidChrAccess, .descriptors = hid_kbm_kb_dscs,
      .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &hid_kb_input_handle },
    { .uuid = &UUID16_HID_REPORT.u, .access_cb = hidChrAccess, .descriptors = hid_kbm_cs_dscs,
      .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &hid_consumer_input_handle },
    { .uuid = &UUID16_HID_REPORT.u, .access_cb = hidChrAccess, .descriptors = hid_kbm_ms_dscs,
      .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &hid_mouse_input_handle },
    { .uuid = &UUID16_HID_REPORT.u, .access_cb = hidChrAccess, .descriptors = hid_kbm_out_dscs,
      .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP },
    { 0 }
};

// Gamepad: Gamepad (ID 1)
static struct ble_gatt_chr_def hid_chars_gamepad[] = {
    { .uuid = &UUID16_HID_INFO.u,   .access_cb = hidChrAccess, .flags = BLE_GATT_CHR_F_READ },
    { .uuid = &UUID16_RPT_MAP.u,    .access_cb = hidChrAccess, .flags = BLE_GATT_CHR_F_READ },
    { .uuid = &UUID16_HID_CTRL.u,   .access_cb = hidChrAccess, .flags = BLE_GATT_CHR_F_WRITE_NO_RSP },
    { .uuid = &UUID16_PROTO_MODE.u, .access_cb = hidChrAccess, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP },
    { .uuid = &UUID16_HID_REPORT.u, .access_cb = hidChrAccess, .descriptors = hid_gp_dscs,
      .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &hid_gamepad_input_handle },
    { 0 }
};

// ---- Per-profile GATT service arrays ----

static const struct ble_gatt_svc_def gatt_svcs_none[] = {
    { .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &NUS_SVC_UUID.u,  .characteristics = nus_chars_with_handle },
    { .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &MIDI_SVC_UUID.u, .characteristics = midi_chars },
    { 0 }
};
static const struct ble_gatt_svc_def gatt_svcs_kb_consumer[] = {
    { .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &NUS_SVC_UUID.u,   .characteristics = nus_chars_with_handle },
    { .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &MIDI_SVC_UUID.u,  .characteristics = midi_chars },
    { .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &UUID16_HID_SVC.u, .characteristics = hid_chars_kb_consumer },
    { 0 }
};
static const struct ble_gatt_svc_def gatt_svcs_mouse[] = {
    { .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &NUS_SVC_UUID.u,   .characteristics = nus_chars_with_handle },
    { .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &MIDI_SVC_UUID.u,  .characteristics = midi_chars },
    { .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &UUID16_HID_SVC.u, .characteristics = hid_chars_mouse },
    { 0 }
};
static const struct ble_gatt_svc_def gatt_svcs_kb_mouse[] = {
    { .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &NUS_SVC_UUID.u,   .characteristics = nus_chars_with_handle },
    { .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &MIDI_SVC_UUID.u,  .characteristics = midi_chars },
    { .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &UUID16_HID_SVC.u, .characteristics = hid_chars_kb_mouse },
    { 0 }
};
static const struct ble_gatt_svc_def gatt_svcs_gamepad[] = {
    { .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &NUS_SVC_UUID.u,   .characteristics = nus_chars_with_handle },
    { .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &MIDI_SVC_UUID.u,  .characteristics = midi_chars },
    { .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &UUID16_HID_SVC.u, .characteristics = hid_chars_gamepad },
    { 0 }
};

#pragma GCC diagnostic pop

// ---- GATT profile switch ----

// Rebuild the GATT table for the selected HID profile.
// Must only be called from the NimBLE host task (e.g., from hidDeviceStart/Stop
// which run on the main dispatcher). Terminates any active HID connection,
// resets all GATT tables, then re-registers NUS + MIDI + selected HID profile.
static void switchGattProfile(HidProfile profile) {
    if (profile == current_hid_profile) return;

    auto bt = bt_singleton;
    LOGGER.info("switchGattProfile: {} -> {}", (int)current_hid_profile, (int)profile);

    // Stop advertising before resetting GATT tables
    ble_gap_adv_stop();

    // Terminate any active HID connection
    if (bt && bt->hidConnHandle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(bt->hidConnHandle, BLE_ERR_REM_USER_CONN_TERM);
        bt->hidConnHandle = BLE_HS_CONN_HANDLE_NONE;
    }

    // Reset all GATT services and re-register
    ble_gatts_reset();
    ble_svc_gap_init();
    ble_svc_gatt_init();

    const struct ble_gatt_svc_def* svcs = gatt_svcs_none;
    switch (profile) {
        case HidProfile::KbConsumer: svcs = gatt_svcs_kb_consumer; active_hid_rpt_map = hid_rpt_map_kb_consumer; active_hid_rpt_map_len = sizeof(hid_rpt_map_kb_consumer); break;
        case HidProfile::Mouse:      svcs = gatt_svcs_mouse;        active_hid_rpt_map = hid_rpt_map_mouse;        active_hid_rpt_map_len = sizeof(hid_rpt_map_mouse);        break;
        case HidProfile::KbMouse:    svcs = gatt_svcs_kb_mouse;     active_hid_rpt_map = hid_rpt_map_kb_mouse;     active_hid_rpt_map_len = sizeof(hid_rpt_map_kb_mouse);     break;
        case HidProfile::Gamepad:    svcs = gatt_svcs_gamepad;      active_hid_rpt_map = hid_rpt_map_gamepad;      active_hid_rpt_map_len = sizeof(hid_rpt_map_gamepad);      break;
        default:                     svcs = gatt_svcs_none;          active_hid_rpt_map = nullptr;                  active_hid_rpt_map_len = 0;                                break;
    }

    int rc = ble_gatts_count_cfg(svcs);
    if (rc == 0) {
        rc = ble_gatts_add_svcs(svcs);
        if (rc != 0) LOGGER.error("switchGattProfile: gatts_add_svcs failed rc={}", rc);
    } else {
        LOGGER.error("switchGattProfile: gatts_count_cfg failed rc={}", rc);
    }

    // Re-apply device name (ble_svc_gap_init resets it)
    ble_svc_gap_device_name_set(CONFIG_TT_DEVICE_NAME);
    ble_att_set_preferred_mtu(BLE_ATT_MTU_MAX);

    // Signal bonded hosts to re-discover the updated GATT database
    ble_svc_gatt_changed(0, 0xFFFF);

    current_hid_profile = profile;
}

// ---- Advertising restart helper ----

// Called (possibly from esp_timer task) to restart advertising for whichever
// service profile is active but not yet connected.
static void advRestartCallback(void* /*arg*/) {
    auto bt = bt_singleton;
    if (bt == nullptr || bt->getRadioState() != RadioState::On) return;
    if (bt->midiActive && bt->midiConnHandle == BLE_HS_CONN_HANDLE_NONE) {
        startAdvertising(&MIDI_SVC_UUID);
    } else if (bt->sppActive && bt->sppConnHandle == BLE_HS_CONN_HANDLE_NONE) {
        startAdvertising(&NUS_SVC_UUID);
    } else if (bt->hidActive && bt->hidConnHandle == BLE_HS_CONN_HANDLE_NONE) {
        startAdvertisingHid(hid_appearance);
    }
}

// Schedule (or immediately execute) an advertising restart.
// delay_us=0 calls the callback inline; >0 uses a one-shot esp_timer so the
// GAP event handler returns quickly without blocking the NimBLE host task.
static void scheduleAdvRestart(std::shared_ptr<Bluetooth> bt, uint64_t delay_us) {
    if (delay_us == 0) {
        advRestartCallback(nullptr);
        return;
    }
    if (bt->advRestartTimer == nullptr) {
        esp_timer_create_args_t args = {};
        args.callback        = advRestartCallback;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name            = "adv_restart";
        esp_timer_create(&args, &bt->advRestartTimer);
    }
    esp_timer_stop(bt->advRestartTimer); // cancel any pending restart (ignore EINVAL if not running)
    esp_timer_start_once(bt->advRestartTimer, delay_us);
}

// ---- GAP scan callback ----

static int gapDiscEventHandler(struct ble_gap_event* event, void* arg) {
    auto bt = bt_singleton;
    if (bt == nullptr) return 0;

    switch (event->type) {
        case BLE_GAP_EVENT_DISC: {
            const auto& disc = event->disc;

            PeerRecord record = {};
            std::memcpy(record.addr.data(), disc.addr.val, BLE_DEV_ADDR_LEN);
            record.rssi = disc.rssi;
            record.paired = false;
            record.connected = false;

            // Parse name from advertisement data
            struct ble_hs_adv_fields fields;
            if (ble_hs_adv_parse_fields(&fields, disc.data, disc.length_data) == 0) {
                if (fields.name != nullptr && fields.name_len > 0) {
                    size_t copy_len = std::min((int)fields.name_len, BT_NAME_MAX);
                    record.name = std::string(reinterpret_cast<const char*>(fields.name), copy_len);
                }
            }

            {
                auto lock = bt->dataMutex.asScopedLock();
                lock.lock();
                // Deduplicate by address; smart-merge fields so we don't
                // clobber a name obtained from ADV_IND with an empty SCAN_RSP.
                bool found = false;
                for (size_t i = 0; i < bt->scanResults.size(); ++i) {
                    if (bt->scanResults[i].addr == record.addr) {
                        if (!record.name.empty()) {
                            bt->scanResults[i].name = record.name;
                        }
                        bt->scanResults[i].rssi = record.rssi;
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    bt->scanResults.push_back(record);
                    bt->scanAddresses.push_back(disc.addr); // save full addr type for name resolution
                }
            }

            // Publish peer found event
            publishEvent(bt, BtEvent::PeerFound);
            break;
        }

        case BLE_GAP_EVENT_DISC_COMPLETE: {
            LOGGER.info("Scan complete (reason={})", event->disc_complete.reason);
            bt->setScanning(false);
            // Resolve names for any devices that didn't broadcast one in their ads.
            // This connects briefly to read the Generic Access Device Name (0x2A00)
            // the same way Windows/Android do in the background.
            resolveNextUnnamedPeer(bt, 0);
            publishEvent(bt, BtEvent::ScanFinished);
            break;
        }

        default:
            break;
    }
    return 0;
}

// ---- GATT Device Name resolution ----
//
// After a scan completes, connect briefly to each device that didn't include
// its name in advertising data and read the Generic Access Device Name
// characteristic (UUID 0x2A00). This is the same technique Windows and Android
// use to resolve BLE device names in the background.
//
// The resolution is sequential: connect → read → disconnect → next device.
// Each connection attempt times out after 1500 ms if the device is unreachable.
//
// We don't attempt resolution if a profile server is active because
// simultaneously initiating a central connection while advertising as a
// peripheral can interfere with the C6 controller over esp_hosted SDIO.

struct NameResCtx {
    std::shared_ptr<Bluetooth> bt;
    size_t idx;                   // index into scanResults being resolved
    std::array<uint8_t, 6> addr; // copy for callback matching (bt->scanResults may shift)
};

static int nameReadCallback(uint16_t conn_handle, const struct ble_gatt_error* error,
                            struct ble_gatt_attr* attr, void* arg) {
    auto* ctx = static_cast<NameResCtx*>(arg);

    if (error->status == 0 && attr != nullptr) {
        uint16_t len = OS_MBUF_PKTLEN(attr->om);
        if (len > 0 && len <= static_cast<uint16_t>(BT_NAME_MAX)) {
            char name_buf[BT_NAME_MAX + 1] = {};
            os_mbuf_copydata(attr->om, 0, len, name_buf);
            auto bt = ctx->bt;
            {
                auto lock = bt->dataMutex.asScopedLock();
                lock.lock();
                for (auto& rec : bt->scanResults) {
                    if (rec.addr == ctx->addr && rec.name.empty()) {
                        rec.name = std::string(name_buf, len);
                        LOGGER.info("Name resolved (idx={}): {}", ctx->idx, rec.name);
                        break;
                    }
                }
            }
            publishEvent(bt, BtEvent::PeerFound);
        }
        return 0; // wait for BLE_HS_EDONE callback
    }

    // BLE_HS_EDONE, ATT error, or timeout — done reading this device
    ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    return 0;
}

static int nameResGapCallback(struct ble_gap_event* event, void* arg) {
    auto* ctx = static_cast<NameResCtx*>(arg);
    auto bt  = ctx->bt;

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                LOGGER.info("Name resolution: connected (idx={} handle={})", ctx->idx, event->connect.conn_handle);
                static const ble_uuid16_t device_name_uuid = BLE_UUID16_INIT(0x2A00);
                int rc = ble_gattc_read_by_uuid(event->connect.conn_handle,
                                                1, 0xFFFF,
                                                &device_name_uuid.u,
                                                nameReadCallback, ctx);
                if (rc != 0) {
                    LOGGER.warn("Name resolution: read_by_uuid failed rc={}", rc);
                    ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                }
            } else {
                LOGGER.info("Name resolution: connect failed (idx={} status={})", ctx->idx, event->connect.status);
                size_t next = ctx->idx + 1;
                delete ctx;
                resolveNextUnnamedPeer(bt, next);
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT: {
            LOGGER.info("Name resolution: disconnected (idx={})", ctx->idx);
            size_t next = ctx->idx + 1;
            delete ctx;
            resolveNextUnnamedPeer(bt, next);
            break;
        }

        default:
            break;
    }
    return 0;
}

// Check scan results for any saved HID host devices and connect to the first one found.
// Called after scan + name resolution complete so ble_gap_connect is available.
// Dispatches to main task for file I/O (settings::load reads .device.properties files).
static void dispatchAutoConnectHidHost(std::shared_ptr<Bluetooth> bt) {
    if (hid_host_ctx) return; // already connecting/connected
    getMainDispatcher().dispatch([bt] {
        if (hid_host_ctx) return;
        // Collect current scan result addresses (under lock)
        std::vector<std::array<uint8_t, 6>> addrs;
        {
            auto lock = bt->dataMutex.asScopedLock();
            lock.lock();
            for (const auto& r : bt->scanResults) addrs.push_back(r.addr);
        }
        // Connect to first saved HID host peer seen in the scan
        for (const auto& addr : addrs) {
            settings::PairedDevice stored;
            if (settings::load(settings::addrToHex(addr), stored) &&
                stored.profileId == BT_PROFILE_HID_HOST &&
                stored.autoConnect) {
                LOGGER.info("HID host: auto-reconnecting to {}", settings::addrToHex(addr));
                hidHostConnect(addr);
                return;
            }
        }
    });
}

static void resolveNextUnnamedPeer(std::shared_ptr<Bluetooth> bt, size_t start_idx) {
    // Skip resolution if a profile server or HID host connection attempt is active —
    // initiating another central connection at the same time would fail with BLE_HS_EALREADY.
    if (bt->midiActive || bt->sppActive || bt->hidActive || hid_host_ctx) {
        LOGGER.info("Name resolution: skipping (server or HID host connection active)");
        dispatchAutoConnectHidHost(bt); // still try auto-connect even if resolution is skipped
        return;
    }

    size_t i = start_idx;
    while (true) {
        ble_addr_t    addr     = {};
        std::array<uint8_t, 6> rec_addr = {};
        bool          found    = false;
        {
            auto lock = bt->dataMutex.asScopedLock();
            lock.lock();
            while (i < bt->scanResults.size()) {
                if (bt->scanResults[i].name.empty()) {
                    addr     = bt->scanAddresses[i];
                    rec_addr = bt->scanResults[i].addr;
                    found    = true;
                    break;
                }
                ++i;
            }
        } // unlock before ble_gap_connect

        if (!found) {
            LOGGER.info("Name resolution: complete (checked {} devices)", i);
            dispatchAutoConnectHidHost(bt);
            return;
        }

        uint8_t own_addr_type;
        ble_hs_id_infer_auto(0, &own_addr_type);

        auto* ctx = new NameResCtx{bt, i, rec_addr};
        int rc = ble_gap_connect(own_addr_type, &addr, 1500, nullptr,
                                 nameResGapCallback, ctx);
        if (rc == 0) {
            return; // nameResGapCallback will continue the chain
        }

        LOGGER.info("Name resolution: ble_gap_connect failed idx={} rc={}, skipping", i, rc);
        delete ctx;
        ++i;
    }
}

// ---- GAP connection event handler ----

static int gapEventHandler(struct ble_gap_event* event, void* arg) {
    auto bt = bt_singleton;
    if (bt == nullptr) return 0;

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                LOGGER.info("Connected (handle={})", event->connect.conn_handle);
                // Do NOT call ble_gap_security_initiate() here.
                //
                // Windows and its BLE MIDI driver initiate encryption themselves.
                // REPEAT_PAIRING+RETRY already triggers a fresh pairing when the peer's
                // LTK is stale (NVS empty after bond delete). Calling security_initiate()
                // here creates a race: RETRY internally calls security_initiate() AND we
                // call it again → two concurrent SM procedures → Windows disconnects.
                //
                // On esp_hosted (P4+C6 via SDIO), BLE_GAP_EVENT_ENC_CHANGE can also
                // arrive BEFORE this CONNECT event, so calling security_initiate() on
                // an already-encrypted link would trigger an unwanted re-pairing.
            } else {
                LOGGER.warn("Connection failed (status={})", event->connect.status);
                // A failed connection attempt stops advertising. Delay the restart by
                // 500 ms so NimBLE can finish cleaning up internal state (SMP tables,
                // connection slots) before the peer retries. Without the delay, rapid
                // Windows retries arrive while NimBLE is still busy → EAGAIN loop.
                scheduleAdvRestart(bt, 500'000);
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT: {
            LOGGER.info("Disconnected (reason={})", event->disconnect.reason);
            uint16_t hdl = event->disconnect.conn.conn_handle;
            bool wasSpp  = (bt->sppConnHandle  == hdl);
            bool wasMidi = (bt->midiConnHandle == hdl);
            bool wasHid  = (bt->hidConnHandle  == hdl);
            if (wasSpp)  bt->sppConnHandle  = BLE_HS_CONN_HANDLE_NONE;
            if (wasMidi) { bt->midiConnHandle = BLE_HS_CONN_HANDLE_NONE; bt->midiUseIndicate = false; }
            if (wasHid)  bt->hidConnHandle  = BLE_HS_CONN_HANDLE_NONE;
            bt->linkEncrypted = false;
            // Restart advertising whenever a service is active and has no live
            // subscription. This covers two cases:
            //   1. A normal data connection ended (wasSpp/wasMidi/wasHid true, handle cleared).
            //   2. A "discovery-only" connection ended without a subscription (e.g. Windows's
            //      first-connect GATT discovery phase). Without re-advertising here, the
            //      second-phase reconnect from the Windows BLE MIDI/HID driver would fail.
            if (bt->midiActive && bt->midiConnHandle == BLE_HS_CONN_HANDLE_NONE) {
                startAdvertising(&MIDI_SVC_UUID);
            } else if (bt->sppActive && bt->sppConnHandle == BLE_HS_CONN_HANDLE_NONE) {
                startAdvertising(&NUS_SVC_UUID);
            } else if (bt->hidActive && bt->hidConnHandle == BLE_HS_CONN_HANDLE_NONE) {
                startAdvertisingHid(hid_appearance);
            }
            break;
        }

        case BLE_GAP_EVENT_SUBSCRIBE:
            LOGGER.info("Subscribe attr={} cur_notify={} cur_indicate={} (nus_tx={} midi_io={})",
                        event->subscribe.attr_handle,
                        (unsigned)event->subscribe.cur_notify,
                        (unsigned)event->subscribe.cur_indicate,
                        nus_tx_handle, midi_io_handle);
            if (event->subscribe.attr_handle != nus_tx_handle &&
                event->subscribe.attr_handle != midi_io_handle &&
                event->subscribe.cur_indicate) {
                // Windows subscribes to GATT Service Changed when it suspects the GATT
                // database changed (e.g. after a firmware flash). Defer the indication
                // until BLE_GAP_EVENT_ENC_CHANGE so it is sent on an encrypted link —
                // Windows ignores Service Changed indications received before encryption.
                // Windows subscribes to GATT Service Changed as standard GATT client
                // init on every new bond. Do NOT respond — Windows performs full GATT
                // discovery on any new bond (including after REPEAT_PAIRING re-pair)
                // and will find our current handles without any hint from us.
                // Sending ble_svc_gatt_changed() here causes a redundant re-discovery
                // that disrupts the Windows BLE MIDI driver connection sequence.
                LOGGER.info("Service Changed subscription (attr={}) — ignoring, Windows discovers on its own",
                            event->subscribe.attr_handle);
            } else if (event->subscribe.attr_handle == nus_tx_handle) {
                if (!bt->sppActive) {
                    // Ignore cross-profile subscription: Windows subscribes to ALL CCCDs on
                    // connect regardless of which profile server is actually running.
                    LOGGER.info("SPP CCCD subscribed but sppActive=false — ignoring");
                    break;
                }
                bt->sppConnHandle = event->subscribe.conn_handle;
                LOGGER.info("SPP client subscribed (nus_tx_handle={})", nus_tx_handle);
                // Dispatch profile update off the NimBLE host task — file I/O on the
                // nimble_host stack causes stack overflows (stringstream + PropertiesFile).
                {
                    struct ble_gap_conn_desc sub_desc = {};
                    if (ble_gap_conn_find(event->subscribe.conn_handle, &sub_desc) == 0) {
                        std::array<uint8_t, 6> sub_addr;
                        std::memcpy(sub_addr.data(), sub_desc.peer_id_addr.val, 6);
                        getMainDispatcher().dispatch([sub_addr] {
                            const auto hex = settings::addrToHex(sub_addr);
                            settings::PairedDevice stored;
                            if (settings::load(hex, stored) && stored.profileId != BT_PROFILE_SPP) {
                                stored.profileId = BT_PROFILE_SPP;
                                settings::save(stored);
                            }
                        });
                    }
                }
            } else if (event->subscribe.attr_handle == midi_io_handle) {
                if ((event->subscribe.cur_notify || event->subscribe.cur_indicate) && !bt->midiActive) {
                    LOGGER.info("MIDI CCCD subscribed but midiActive=false — ignoring");
                    break;
                }
                if (event->subscribe.cur_notify || event->subscribe.cur_indicate) {
                    bt->midiConnHandle  = event->subscribe.conn_handle;
                    bt->midiUseIndicate = (event->subscribe.cur_indicate != 0);
                    LOGGER.info("MIDI client subscribed (midi_io_handle={} indicate={})",
                                midi_io_handle, bt->midiUseIndicate);
                    // Dispatch profile update off the NimBLE host task (same reason as SPP above).
                    {
                        struct ble_gap_conn_desc sub_desc = {};
                        if (ble_gap_conn_find(event->subscribe.conn_handle, &sub_desc) == 0) {
                            std::array<uint8_t, 6> sub_addr;
                            std::memcpy(sub_addr.data(), sub_desc.peer_id_addr.val, 6);
                            getMainDispatcher().dispatch([sub_addr] {
                                const auto hex = settings::addrToHex(sub_addr);
                                settings::PairedDevice stored;
                                if (settings::load(hex, stored) && stored.profileId != BT_PROFILE_MIDI) {
                                    stored.profileId = BT_PROFILE_MIDI;
                                    settings::save(stored);
                                }
                            });
                        }
                    }
                    // Send MIDI Active Sensing (0xFE) immediately so Windows BLE MIDI
                    // driver doesn't time out waiting for the first MIDI packet.
                    // BLE MIDI packet: [header=0x80][timestamp=0x80][status=0xFE]
                    static const uint8_t active_sensing_pkt[3] = { 0x80, 0x80, 0xFE };
                    struct os_mbuf* as_om = ble_hs_mbuf_from_flat(active_sensing_pkt, 3);
                    if (as_om != nullptr) {
                        int as_rc = bt->midiUseIndicate
                            ? ble_gatts_indicate_custom(bt->midiConnHandle, midi_io_handle, as_om)
                            : ble_gatts_notify_custom(bt->midiConnHandle, midi_io_handle, as_om);
                        LOGGER.info("Active Sensing (subscribe) rc={}", as_rc);
                    }
                } else {
                    // Unsubscribe — clear the connection handle
                    bt->midiConnHandle  = BLE_HS_CONN_HANDLE_NONE;
                    bt->midiUseIndicate = false;
                    LOGGER.info("MIDI client unsubscribed");
                }
            } else if (event->subscribe.cur_notify &&
                       (event->subscribe.attr_handle == hid_kb_input_handle ||
                        event->subscribe.attr_handle == hid_consumer_input_handle ||
                        event->subscribe.attr_handle == hid_mouse_input_handle ||
                        event->subscribe.attr_handle == hid_gamepad_input_handle)) {
                const char* report_name =
                    (event->subscribe.attr_handle == hid_kb_input_handle)       ? "keyboard" :
                    (event->subscribe.attr_handle == hid_consumer_input_handle)  ? "consumer" :
                    (event->subscribe.attr_handle == hid_mouse_input_handle)     ? "mouse"    :
                    (event->subscribe.attr_handle == hid_gamepad_input_handle)   ? "gamepad"  : "unknown";
                if (!bt->hidActive) {
                    LOGGER.info("HID CCCD subscribed ({}) but hidActive=false — ignoring", report_name);
                    break;
                }
                LOGGER.info("HID CCCD subscribed: {} (attr={} conn={})",
                            report_name, event->subscribe.attr_handle,
                            event->subscribe.conn_handle);
                if (bt->hidConnHandle == BLE_HS_CONN_HANDLE_NONE) {
                    bt->hidConnHandle = event->subscribe.conn_handle;
                }
            }
            break;

        case BLE_GAP_EVENT_MTU:
            LOGGER.info("MTU updated (conn={} mtu={})",
                        event->mtu.conn_handle, event->mtu.value);
            break;

        case BLE_GAP_EVENT_CONN_UPDATE: {
            struct ble_gap_conn_desc desc = {};
            ble_gap_conn_find(event->conn_update.conn_handle, &desc);
            LOGGER.info("Conn params updated (status={} itvl={} latency={} timeout={})",
                        event->conn_update.status,
                        desc.conn_itvl,
                        desc.conn_latency,
                        desc.supervision_timeout);
            break;
        }

        case BLE_GAP_EVENT_CONN_UPDATE_REQ:
            // Accept whatever the central requests
            *event->conn_update_req.self_params = *event->conn_update_req.peer_params;
            return 0;

        case BLE_GAP_EVENT_ENC_CHANGE:
            LOGGER.info("Encryption changed (conn={} status={})",
                        event->enc_change.conn_handle, event->enc_change.status);
            if (event->enc_change.status == 0) {
                bt->linkEncrypted = true;
                // Persist the peer in our own storage on first successful bond.
                // NimBLE stores the LTK in NVS via ble_store_config_init(); we store
                // a separate .device.properties file so getPairedPeers() can enumerate.
                // Dispatch file I/O off the NimBLE host task to avoid stack overflow
                // (stringstream + PropertiesFile routines consume too much stack).
                struct ble_gap_conn_desc desc = {};
                if (ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0) {
                    std::array<uint8_t, 6> peer_addr;
                    std::memcpy(peer_addr.data(), desc.peer_id_addr.val, 6);
                    int profile = bt->midiActive ? BT_PROFILE_MIDI
                                : bt->sppActive  ? BT_PROFILE_SPP
                                                 : BT_PROFILE_HID_HOST;
                    getMainDispatcher().dispatch([bt, peer_addr, profile] {
                        const auto addr_hex = settings::addrToHex(peer_addr);
                        if (!settings::contains(addr_hex)) {
                            settings::PairedDevice device;
                            device.addr      = peer_addr;
                            device.name      = "";
                            device.autoConnect = true;
                            device.profileId = profile;
                            if (settings::save(device)) {
                                LOGGER.info("Saved paired peer {} (profile={})", addr_hex, profile);
                                publishEvent(bt, BtEvent::PairSuccess);
                            }
                        }
                    });
                }
            }
            // Re-send Active Sensing now that the link is encrypted.
            // Windows BLE MIDI ignores pre-encryption indications; sending here
            // resets its idle timer so it doesn't disconnect after ~5 seconds.
            if (event->enc_change.status == 0 &&
                bt->midiConnHandle == event->enc_change.conn_handle) {
                static const uint8_t as_pkt[3] = { 0x80, 0x80, 0xFE };
                struct os_mbuf* om = ble_hs_mbuf_from_flat(as_pkt, 3);
                if (om != nullptr) {
                    int rc = bt->midiUseIndicate
                        ? ble_gatts_indicate_custom(bt->midiConnHandle, midi_io_handle, om)
                        : ble_gatts_notify_custom(bt->midiConnHandle, midi_io_handle, om);
                    LOGGER.info("Active Sensing (post-enc) rc={}", rc);
                }
            }
            break;

        case BLE_GAP_EVENT_PASSKEY_ACTION: {
            LOGGER.info("Passkey action requested (conn={} action={})",
                        event->passkey.conn_handle, event->passkey.params.action);
            // "Just Works" — no passkey needed; inject IOACT_NONE to unblock pairing.
            struct ble_sm_io pkey = {};
            pkey.action = event->passkey.params.action;
            if (pkey.action == BLE_SM_IOACT_NONE) {
                ble_sm_inject_io(event->passkey.conn_handle, &pkey);
            }
            break;
        }

        case BLE_GAP_EVENT_REPEAT_PAIRING: {
            // Stale bond: the peer's LTK doesn't match what NimBLE has stored
            // (e.g. device was reflashed, NVS cleared). Always delete the stale
            // bond entry so the next fresh connection can pair cleanly.
            LOGGER.info("Repeat pairing (conn={} encrypted={})",
                        event->repeat_pairing.conn_handle, bt->linkEncrypted);
            struct ble_gap_conn_desc desc;
            if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
                ble_store_util_delete_peer(&desc.peer_id_addr);
                if (!bt->linkEncrypted) {
                    // Also remove our .device.properties record — it will be recreated
                    // by the ENC_CHANGE handler after the fresh pairing completes.
                    std::array<uint8_t, 6> peer_addr;
                    std::memcpy(peer_addr.data(), desc.peer_id_addr.val, 6);
                    settings::remove(settings::addrToHex(peer_addr));
                }
            }
            // If encryption already succeeded (ENC_CHANGE status=0 already fired),
            // the connection is working — return IGNORE instead of RETRY.
            // RETRY would start a 30-second SMP pairing timer; the peer (Windows)
            // won't respond to a new Pair Request while it thinks the session is
            // already encrypted, so the timer always expires → BLE_HS_ETIMEOUT
            // (status=13) → forced disconnect ~30 s after MIDI subscription.
            // IGNORE lets the current encrypted session continue uninterrupted.
            if (bt->linkEncrypted) {
                LOGGER.info("Repeat pairing: link already encrypted — ignoring, session continues");
                return BLE_GAP_REPEAT_PAIRING_IGNORE;
            }
            return BLE_GAP_REPEAT_PAIRING_RETRY;
        }

        default:
            break;
    }
    return 0;
}

// ---- GATT characteristic access callbacks ----

static int nusChrAccess(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt* ctxt, void* arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        LOGGER.info("NUS RX {} bytes", len);
        auto bt = bt_singleton;
        if (bt != nullptr && len > 0) {
            std::vector<uint8_t> packet(len);
            os_mbuf_copydata(ctxt->om, 0, len, packet.data());
            {
                auto lock = bt->dataMutex.asScopedLock();
                lock.lock();
                bt->sppRxQueue.push_back(std::move(packet));
                // Cap at 16 packets to bound memory use on a slow consumer
                while (bt->sppRxQueue.size() > 16) {
                    bt->sppRxQueue.pop_front();
                }
            }
            publishEvent(bt, BtEvent::SppDataReceived);
        }
    }
    return 0;
}

static int midiChrAccess(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt* ctxt, void* arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        LOGGER.info("MIDI RX {} bytes", len);
        auto bt = bt_singleton;
        if (bt != nullptr && len > 0) {
            std::vector<uint8_t> packet(len);
            os_mbuf_copydata(ctxt->om, 0, len, packet.data());
            {
                auto lock = bt->dataMutex.asScopedLock();
                lock.lock();
                bt->midiRxQueue.push_back(std::move(packet));
                while (bt->midiRxQueue.size() > 16) bt->midiRxQueue.pop_front();
            }
            publishEvent(bt, BtEvent::MidiDataReceived);
        }
    }
    return 0;
}

// ---- HID GATT callbacks ----

// Report Reference descriptor (0x2908): reads the 2-byte {Report ID, Report Type} data
// stored in the descriptor's `arg` pointer (one of the hid_rpt_ref_* arrays above).
static int hidDscAccess(uint16_t /*conn_handle*/, uint16_t /*attr_handle*/,
                        struct ble_gatt_access_ctxt* ctxt, void* arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC) {
        const uint8_t* data = static_cast<const uint8_t*>(arg);
        int rc = os_mbuf_append(ctxt->om, data, 2);
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

// HID characteristic access — handles READ/WRITE for all chars in the HID service.
// Multiple Input Report characteristics share UUID 0x2A4D; they are distinguished by
// attr_handle (set at GATTS registration time via val_handle pointers).
static int hidChrAccess(uint16_t /*conn_handle*/, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt* ctxt, void* /*arg*/) {
    uint16_t uuid16 = ble_uuid_u16(ctxt->chr->uuid);

    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR: {
            if (uuid16 == 0x2A4A) {
                // HID Information: bcdHID=0x0111 (v1.11), bCountryCode=0x00, Flags=0x02 (NormallyConnectable)
                static const uint8_t hid_info[4] = { 0x11, 0x01, 0x00, 0x02 };
                int rc = os_mbuf_append(ctxt->om, hid_info, sizeof(hid_info));
                return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            if (uuid16 == 0x2A4B) {
                // Report Map — serve the map for the currently active HID profile
                if (active_hid_rpt_map == nullptr || active_hid_rpt_map_len == 0)
                    return BLE_ATT_ERR_UNLIKELY;
                int rc = os_mbuf_append(ctxt->om, active_hid_rpt_map, active_hid_rpt_map_len);
                return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            if (uuid16 == 0x2A4E) {
                // Protocol Mode
                int rc = os_mbuf_append(ctxt->om, &hid_protocol_mode, 1);
                return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            if (uuid16 == 0x2A4D) {
                // Input / Output Reports — return zeros (no buffered state)
                static const uint8_t zeros[8] = {};
                size_t report_len = 8; // default / keyboard / gamepad
                if (attr_handle == hid_consumer_input_handle)  report_len = 2;
                else if (attr_handle == hid_mouse_input_handle) report_len = 4;
                int rc = os_mbuf_append(ctxt->om, zeros, report_len);
                return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            return BLE_ATT_ERR_UNLIKELY;
        }

        case BLE_GATT_ACCESS_OP_WRITE_CHR: {
            if (uuid16 == 0x2A4C) {
                // HID Control Point (suspend / exit-suspend) — no action needed
                return 0;
            }
            if (uuid16 == 0x2A4E) {
                // Protocol Mode write: 0x00=Boot, 0x01=Report
                if (OS_MBUF_PKTLEN(ctxt->om) >= 1) {
                    os_mbuf_copydata(ctxt->om, 0, 1, &hid_protocol_mode);
                    LOGGER.info("HID Protocol Mode -> {}", hid_protocol_mode);
                }
                return 0;
            }
            if (uuid16 == 0x2A4D) {
                // Output Report: keyboard LEDs (Num/Caps/Scroll lock).
                // Bit 0=Num, 1=Caps, 2=Scroll, 3=Compose, 4=Kana.
                if (OS_MBUF_PKTLEN(ctxt->om) >= 1) {
                    uint8_t leds = 0;
                    os_mbuf_copydata(ctxt->om, 0, 1, &leds);
                    LOGGER.info("HID keyboard LED state: 0x{:02x}", leds);
                }
                return 0;
            }
            return BLE_ATT_ERR_UNLIKELY;
        }

        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

// ---- NimBLE host sync / reset callbacks ----

static void onSync() {
    LOGGER.info("NimBLE host synced (nus_tx={} midi_io={} hid_kb={} hid_cs={} hid_ms={} hid_gp={})",
                nus_tx_handle, midi_io_handle,
                hid_kb_input_handle, hid_consumer_input_handle,
                hid_mouse_input_handle, hid_gamepad_input_handle);
    auto bt = bt_singleton;
    if (bt == nullptr) return;

    bt->pendingResetCount.store(0);

    // Set random address
    uint8_t own_addr_type;
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        LOGGER.error("ensure_addr failed (rc={})", rc);
    }
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        LOGGER.error("infer addr type failed (rc={})", rc);
    }

    bt->setRadioState(RadioState::On);
    publishEvent(bt, BtEvent::RadioStateOn);

    // Auto-start profile servers that were active before the last reboot.
    // This lets bonded centrals (phone, PC) reconnect without user intervention.
    if (settings::shouldMidiAutoStart()) {
        LOGGER.info("Auto-starting MIDI server");
        midiStart(nullptr);
        return; // midiStart calls startAdvertising — no need for name-only advert below
    }
    if (settings::shouldSppAutoStart()) {
        LOGGER.info("Auto-starting SPP server");
        sppStart(nullptr);
        return;
    }

    // Begin advertising immediately (name-only) so the device is visible in
    // Windows "Add a device → Everything else" and Android BT settings as
    // soon as the radio is up, even before any profile server is started.
    startAdvertising(nullptr);
}

static void onReset(int reason) {
    LOGGER.warn("NimBLE host reset (reason={})", reason);
    auto bt = bt_singleton;
    if (bt == nullptr) return;

    if (bt->getRadioState() == RadioState::OnPending) {
        int count = bt->pendingResetCount.fetch_add(1) + 1;
        if (count == 3) {
            LOGGER.error("BT controller unresponsive after 3 resets — giving up");
            // Dispatch stop from main task; can't call nimble_port_stop() from within the host task
            getMainDispatcher().dispatch([bt] { dispatchDisable(bt); });
        }
    }
}

static void bleHostTask(void* param) {
    LOGGER.info("BLE host task started");
    nimble_port_run();
    // nimble_port_deinit() is called by dispatchDisable() after nimble_port_stop() returns.
    // In IDF 5.5+, nimble_port_stop() blocks until run() exits, so deinit happens in the
    // caller after stop() returns — not here (calling it here races with the stop() semaphore pend).
    nimble_port_freertos_deinit();
}

// ---- Advertise helper ----

// svcUuid: 128-bit service UUID to include in the scan response so that
// profile-specific clients (e.g. BLE MIDI apps, nRF Connect) can find the
// Build raw advertising data with the 128-bit service UUID so that
// UUID-filtered scanners (MIDI apps, nRF Connect UUID filter, etc.) can
// find the device.  Raw bytes bypass any ble_hs_adv_fields serialisation
// quirks.
//
// Packet layout (21 bytes):
//   [0]    0x02  length
//   [1]    0x01  AD type: Flags
//   [2]    0x06  LE General Discoverable | BR/EDR Not Supported
//   [3]    0x11  length (17)
//   [4]    0x07  AD type: Complete 128-bit UUID list
//   [5-20]       UUID in little-endian (LSB first)
// When svcUuid != nullptr: flags + UUID128 + shortened name (fits in 31 bytes).
//   No scan response — avoids a second HCI command through esp_hosted SDIO which
//   has been observed to interfere with Android UUID scan-filter matching.
//   The shortened name (≤8 chars of AD type 0x08) sits in the remaining 10 bytes
//   so Windows passive scanners can label the device in "Add a device".
// When nullptr: name-only primary (generic discoverability when no service is active).
static void startAdvertising(const ble_uuid128_t* svcUuid) {
    ble_gap_adv_stop(); // BLE_HS_EALREADY is fine — means not currently advertising

    int rc;
    if (svcUuid != nullptr) {
        // Primary: flags + complete UUID128 + shortened local name.
        // Use ble_gap_adv_set_fields (NimBLE's own serializer) so the UUID bytes
        // are in exactly the format Android's BLE scan filter expects.
        const char* name = ble_svc_gap_device_name();
        uint8_t name_len = static_cast<uint8_t>(strlen(name));
        uint8_t short_len = (name_len > 8) ? 8 : name_len;

        ble_uuid128_t uuid_copy = *svcUuid;
        struct ble_hs_adv_fields fields;
        std::memset(&fields, 0, sizeof(fields));
        fields.flags             = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
        fields.uuids128          = &uuid_copy;
        fields.num_uuids128      = 1;
        fields.uuids128_is_complete = 1;
        fields.name              = reinterpret_cast<const uint8_t*>(name);
        fields.name_len          = short_len;
        fields.name_is_complete  = 0; // AD type 0x08 = Shortened Local Name

        rc = ble_gap_adv_set_fields(&fields);
        if (rc != 0) {
            // Shouldn't happen (3+18+10=31 bytes fits), but fall back without name
            LOGGER.warn("startAdvertising: set_fields with name failed rc={}, retrying", rc);
            fields.name     = nullptr;
            fields.name_len = 0;
            rc = ble_gap_adv_set_fields(&fields);
            if (rc != 0) {
                LOGGER.error("startAdvertising: set_fields failed rc={}", rc);
                return;
            }
        }
        // Scan response: complete local name via structured API (Windows active scan / nRF Connect)
        struct ble_hs_adv_fields rsp;
        std::memset(&rsp, 0, sizeof(rsp));
        rsp.name             = reinterpret_cast<const uint8_t*>(name);
        rsp.name_len         = name_len;
        rsp.name_is_complete = 1;
        rc = ble_gap_adv_rsp_set_fields(&rsp);
        if (rc != 0) {
            LOGGER.warn("startAdvertising: rsp_set_fields rc={} (non-fatal)", rc);
        }

        LOGGER.info("startAdvertising: UUID mode (uuid[0..3]={:02x}{:02x}{:02x}{:02x})",
                    svcUuid->value[0], svcUuid->value[1],
                    svcUuid->value[2], svcUuid->value[3]);
    } else {
        // Primary: flags + complete local name
        struct ble_hs_adv_fields fields;
        std::memset(&fields, 0, sizeof(fields));
        fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
        const char* name = ble_svc_gap_device_name();
        fields.name = reinterpret_cast<const uint8_t*>(name);
        fields.name_len = strlen(name);
        fields.name_is_complete = 1;
        rc = ble_gap_adv_set_fields(&fields);
        if (rc != 0) {
            LOGGER.error("startAdvertising: set fields failed rc={}", rc);
            return;
        }
        LOGGER.info("startAdvertising: name-only mode");
    }

    struct ble_gap_adv_params adv_params;
    std::memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = 160; // 100 ms
    adv_params.itvl_max = 240; // 150 ms

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, nullptr, BLE_HS_FOREVER,
                           &adv_params, gapEventHandler, nullptr);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        LOGGER.error("startAdvertising: adv start failed rc={}", rc);
    } else {
        LOGGER.info("startAdvertising: OK");
    }
}

// ---- HID advertising ----

// Advertise as a BLE HID device (appearance=keyboard 0x03C1, UUID 0x1812).
// Windows and Android discover HID peripherals by the 16-bit service UUID.
// The appearance value causes Windows to show a keyboard icon in "Add a device".
// appearance: BLE GAP Appearance UUID16 value.
//   0x03C0=Generic HID, 0x03C1=Keyboard, 0x03C2=Mouse, 0x03C4=Gamepad, 0x03C5=Joystick
static void startAdvertisingHid(uint16_t appearance) {
    ble_gap_adv_stop();

    const char* name = ble_svc_gap_device_name();
    uint8_t name_len = static_cast<uint8_t>(strlen(name));
    uint8_t short_len = (name_len > 8) ? 8 : name_len;

    static const ble_uuid16_t hid_uuid16 = BLE_UUID16_INIT(0x1812);

    struct ble_hs_adv_fields fields;
    std::memset(&fields, 0, sizeof(fields));
    fields.flags                = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.appearance           = appearance;
    fields.appearance_is_present = 1;
    fields.uuids16              = &hid_uuid16;
    fields.num_uuids16          = 1;
    fields.uuids16_is_complete  = 1;
    fields.name                 = reinterpret_cast<const uint8_t*>(name);
    fields.name_len             = short_len;
    fields.name_is_complete     = 0; // AD type 0x08 Shortened Local Name

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        // Retry without name if payload is too long
        fields.name     = nullptr;
        fields.name_len = 0;
        rc = ble_gap_adv_set_fields(&fields);
        if (rc != 0) {
            LOGGER.error("startAdvertisingHid: set_fields failed rc={}", rc);
            return;
        }
    }

    // Scan response: complete local name
    struct ble_hs_adv_fields rsp;
    std::memset(&rsp, 0, sizeof(rsp));
    rsp.name             = reinterpret_cast<const uint8_t*>(name);
    rsp.name_len         = name_len;
    rsp.name_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&rsp);
    if (rc != 0) {
        LOGGER.warn("startAdvertisingHid: rsp_set_fields rc={} (non-fatal)", rc);
    }

    struct ble_gap_adv_params adv_params;
    std::memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min  = 160; // 100 ms
    adv_params.itvl_max  = 240; // 150 ms

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, nullptr, BLE_HS_FOREVER,
                           &adv_params, gapEventHandler, nullptr);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        LOGGER.error("startAdvertisingHid: adv start failed rc={}", rc);
    } else {
        LOGGER.info("startAdvertisingHid: OK");
    }
}

// ---- Dispatch helpers ----

static void dispatchEnable(std::shared_ptr<Bluetooth> bt) {
    LOGGER.info("dispatchEnable()");

    if (bt->getRadioState() != RadioState::Off) {
        LOGGER.warn("Cannot enable from current state");
        return;
    }

    bt->setRadioState(RadioState::OnPending);
    publishEvent(bt, BtEvent::RadioStateOnPending);

    // NimBLE init
    int rc = nimble_port_init();
    if (rc != 0) {
        LOGGER.error("nimble_port_init failed (rc={})", rc);
        bt->setRadioState(RadioState::Off);
        publishEvent(bt, BtEvent::RadioStateOff);
        return;
    }

    ble_hs_cfg.sync_cb        = onSync;
    ble_hs_cfg.reset_cb       = onReset;
    // Required for bonding key storage: without this, ble_store_write_peer_sec()
    // has no backing store and pairing silently fails (LTK is never saved, the host
    // disconnects, and the device never appears in OS-level MIDI/device lists).
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // Security Manager: "Just Works" pairing with bonding.
    ble_hs_cfg.sm_io_cap         = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding        = 1;
    ble_hs_cfg.sm_mitm           = 1; // Set per reference impl; Just Works still used (NO_IO)
    ble_hs_cfg.sm_sc             = 1; // LE Secure Connections preferred
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    // Initialise the NVS-backed bond store and wire up its callbacks.
    // ble_store_config_init() sets up the NVS namespace; without it the write
    // callback silently discards bonds and REPEAT_PAIRING fires on every reconnect.
    ble_store_config_init();
    ble_hs_cfg.store_read_cb   = ble_store_config_read;
    ble_hs_cfg.store_write_cb  = ble_store_config_write;
    ble_hs_cfg.store_delete_cb = ble_store_config_delete;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Register base GATT services (NUS + MIDI; HID is added by switchGattProfile when started)
    current_hid_profile    = HidProfile::None;
    active_hid_rpt_map     = nullptr;
    active_hid_rpt_map_len = 0;
    rc = ble_gatts_count_cfg(gatt_svcs_none);
    if (rc != 0) {
        LOGGER.error("gatts_count_cfg failed (rc={})", rc);
    } else {
        rc = ble_gatts_add_svcs(gatt_svcs_none);
        if (rc != 0) {
            LOGGER.error("gatts_add_svcs failed (rc={})", rc);
        }
    }

    // Set device name
    ble_svc_gap_device_name_set(CONFIG_TT_DEVICE_NAME);

    // Advertise maximum MTU so hosts negotiate efficiently
    ble_att_set_preferred_mtu(BLE_ATT_MTU_MAX);

    // Start NimBLE host task (onSync will fire when ready)
    nimble_port_freertos_init(bleHostTask);
    // State transitions to On happen in onSync callback
}

static void dispatchDisable(std::shared_ptr<Bluetooth> bt) {
    LOGGER.info("dispatchDisable()");

    if (bt->getRadioState() == RadioState::Off) {
        LOGGER.warn("Already off");
        return;
    }

    bt->setRadioState(RadioState::OffPending);
    publishEvent(bt, BtEvent::RadioStateOffPending);

    // Signal the NimBLE event loop to exit. IDF 5.5+ makes nimble_port_stop() blocking — it
    // queues the stop event then waits on ble_hs_stop_sem until nimble_port_run() exits.
    // Safe to call nimble_port_deinit() immediately after it returns.
    // Do NOT call ble_gap_adv_stop()/disc_cancel() here — if the controller is unresponsive they
    // generate more HCI timeouts which trigger more onReset callbacks before the stop takes effect.
    nimble_port_stop();
    nimble_port_deinit();

    bt->sppConnHandle  = BLE_HS_CONN_HANDLE_NONE;
    bt->sppActive      = false;
    bt->midiConnHandle = BLE_HS_CONN_HANDLE_NONE;
    bt->midiActive     = false;
    bt->hidConnHandle  = BLE_HS_CONN_HANDLE_NONE;
    bt->hidActive      = false;
    bt->linkEncrypted  = false;
    current_hid_profile    = HidProfile::None;
    active_hid_rpt_map     = nullptr;
    active_hid_rpt_map_len = 0;
    bt->pendingResetCount.store(0);

    // Stop and release esp_timers so the Bluetooth singleton can be destroyed cleanly.
    if (bt->midiKeepaliveTimer != nullptr) {
        esp_timer_stop(bt->midiKeepaliveTimer);
        esp_timer_delete(bt->midiKeepaliveTimer);
        bt->midiKeepaliveTimer = nullptr;
    }
    if (bt->advRestartTimer != nullptr) {
        esp_timer_stop(bt->advRestartTimer);
        esp_timer_delete(bt->advRestartTimer);
        bt->advRestartTimer = nullptr;
    }

    bt->setRadioState(RadioState::Off);
    publishEvent(bt, BtEvent::RadioStateOff);
}

static void dispatchScanStart(std::shared_ptr<Bluetooth> bt) {
    LOGGER.info("dispatchScanStart()");

    if (bt->getRadioState() != RadioState::On) {
        LOGGER.warn("Cannot scan when radio is off");
        return;
    }

    if (bt->isScanning()) {
        LOGGER.warn("Already scanning");
        return;
    }

    {
        auto lock = bt->dataMutex.asScopedLock();
        lock.lock();
        bt->scanResults.clear();
        bt->scanAddresses.clear();
    }

    struct ble_gap_disc_params disc_params = {};
    disc_params.passive = 0;
    disc_params.filter_duplicates = 1;
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    uint8_t own_addr_type;
    ble_hs_id_infer_auto(0, &own_addr_type);

    int rc = ble_gap_disc(own_addr_type, 5000, &disc_params, gapDiscEventHandler, nullptr);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        LOGGER.error("ble_gap_disc failed (rc={})", rc);
        return;
    }

    bt->setScanning(true);
    publishEvent(bt, BtEvent::ScanStarted);
}

static void dispatchScanStop(std::shared_ptr<Bluetooth> bt) {
    ble_gap_disc_cancel();
    bt->setScanning(false);
    publishEvent(bt, BtEvent::ScanFinished);
}

// ---- Event publishing ----

static void publishEvent(std::shared_ptr<Bluetooth> bt, BtEvent event) {
    bt->pubsub->publish(event);
}

// ---- Public service API ----

std::shared_ptr<PubSub<BtEvent>> getPubsub() {
    auto bt = bt_singleton;
    check(bt != nullptr, "Bluetooth service not running");
    return bt->pubsub;
}

RadioState getRadioState() {
    auto bt = bt_singleton;
    if (bt == nullptr) return RadioState::Off;
    return bt->getRadioState();
}

void setEnabled(bool enabled) {
    auto bt = bt_singleton;
    if (bt == nullptr) return;
    if (enabled) {
        getMainDispatcher().dispatch([bt] { dispatchEnable(bt); });
    } else {
        getMainDispatcher().dispatch([bt] { dispatchDisable(bt); });
    }
}

void scanStart() {
    auto bt = bt_singleton;
    if (bt == nullptr) return;
    getMainDispatcher().dispatch([bt] { dispatchScanStart(bt); });
}

void scanStop() {
    auto bt = bt_singleton;
    if (bt == nullptr) return;
    getMainDispatcher().dispatch([bt] { dispatchScanStop(bt); });
}

bool isScanning() {
    auto bt = bt_singleton;
    if (bt == nullptr) return false;
    return bt->isScanning();
}

std::vector<PeerRecord> getScanResults() {
    auto bt = bt_singleton;
    if (bt == nullptr) return {};
    auto lock = bt->dataMutex.asScopedLock();
    lock.lock();
    auto results = bt->scanResults;
    // Mark the HID host connected peer (if any) in the scan list
    if (hidHostIsConnectedImpl() && hid_host_ctx) {
        for (auto& r : results) {
            if (r.addr == hid_host_ctx->peerAddr) {
                r.connected  = true;
                r.profileId  = BT_PROFILE_HID_HOST;
                break;
            }
        }
    }
    return results;
}

std::vector<PeerRecord> getPairedPeers() {
    auto stored = settings::loadAll();
    std::vector<PeerRecord> result;
    result.reserve(stored.size());
    for (const auto& device : stored) {
        PeerRecord record;
        record.addr = device.addr;
        record.name = device.name;
        record.rssi = 0;
        record.paired   = true;
        record.profileId = device.profileId;
        // Mark as connected if this is the active HID host peer
        record.connected = hidHostIsConnectedImpl() && hid_host_ctx &&
                           hid_host_ctx->peerAddr == device.addr;
        result.push_back(std::move(record));
    }
    return result;
}

void pair(const std::array<uint8_t, 6>& addr) {
    LOGGER.info("pair()");
    auto bt = bt_singleton;
    if (bt == nullptr) return;
    ble_addr_t ble_addr;
    ble_addr.type = BLE_ADDR_PUBLIC;
    std::memcpy(ble_addr.val, addr.data(), 6);
    // NimBLE pairing is triggered during connection; initiate security
    uint16_t conn_handle;
    if (ble_gap_conn_find_by_addr(&ble_addr, nullptr) == 0) {
        // If already connected, initiate security
        // ble_gap_security_initiate(conn_handle);
    }
}

void unpair(const std::array<uint8_t, 6>& addr) {
    LOGGER.info("unpair()");
    auto hex = settings::addrToHex(addr);
    settings::remove(hex);
    ble_addr_t ble_addr;
    ble_addr.type = BLE_ADDR_PUBLIC;
    std::memcpy(ble_addr.val, addr.data(), 6);
    ble_store_util_delete_peer(&ble_addr);
}

void connect(const std::array<uint8_t, 6>& addr, int profileId) {
    LOGGER.info("connect(profile={})", profileId);
    auto bt = bt_singleton;
    if (bt == nullptr) return;

    if (profileId == BT_PROFILE_HID_HOST) {
        hidHostConnect(addr);
    } else if (profileId == BT_PROFILE_HID_DEVICE) {
        hidDeviceStart();
    } else if (profileId == BT_PROFILE_SPP) {
        startAdvertising(&NUS_SVC_UUID);
    } else if (profileId == BT_PROFILE_MIDI) {
        startAdvertising(&MIDI_SVC_UUID);
    }
}

void disconnect(const std::array<uint8_t, 6>& addr, int profileId) {
    LOGGER.info("disconnect(profile={})", profileId);
    auto bt = bt_singleton;
    if (bt == nullptr) return;

    if (profileId == BT_PROFILE_HID_HOST) {
        hidHostDisconnect();
    } else if (profileId == BT_PROFILE_HID_DEVICE) {
        hidDeviceStop();
    } else if (profileId == BT_PROFILE_SPP && bt->sppConnHandle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(bt->sppConnHandle, BLE_ERR_REM_USER_CONN_TERM);
        bt->sppConnHandle = BLE_HS_CONN_HANDLE_NONE;
    } else if (profileId == BT_PROFILE_MIDI && bt->midiConnHandle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(bt->midiConnHandle, BLE_ERR_REM_USER_CONN_TERM);
        bt->midiConnHandle = BLE_HS_CONN_HANDLE_NONE;
    }
}

bool hidHostIsConnected() {
    return hidHostIsConnectedImpl();
}

bool isProfileSupported(int profileId) {
    // All BLE profiles are supported on NimBLE
    return profileId == BT_PROFILE_HID_HOST ||
           profileId == BT_PROFILE_HID_DEVICE ||
           profileId == BT_PROFILE_SPP ||
           profileId == BT_PROFILE_MIDI;
}

// ---- Profile sub-API implementations ----

// Serial (BLE SPP)
static error_t sppStart(struct Device* device) {
    auto bt = bt_singleton;
    if (bt == nullptr) return ERROR_INVALID_STATE;
    bt->sppActive = true;
    startAdvertising(&NUS_SVC_UUID);
    return ERROR_NONE;
}

static error_t sppStop(struct Device* device) {
    auto bt = bt_singleton;
    if (bt == nullptr) return ERROR_NONE;
    bt->sppActive = false;
    if (bt->sppConnHandle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(bt->sppConnHandle, BLE_ERR_REM_USER_CONN_TERM);
        bt->sppConnHandle = BLE_HS_CONN_HANDLE_NONE;
    }
    // Do NOT restart advertising after a user-initiated stop.
    // Restarting name-only advertising causes bonded Windows hosts to auto-reconnect
    // in a tight loop (connect → discover → disconnect → repeat).
    // Advertising will resume when the user starts a new profile server.
    if (!bt->midiActive && !bt->hidActive) {
        ble_gap_adv_stop();
    }
    return ERROR_NONE;
}

static error_t sppWrite(struct Device* device, const uint8_t* data, size_t len, size_t* written) {
    auto bt = bt_singleton;
    if (bt == nullptr || bt->sppConnHandle == BLE_HS_CONN_HANDLE_NONE) {
        *written = 0;
        return ERROR_INVALID_STATE;
    }
    struct os_mbuf* om = ble_hs_mbuf_from_flat(data, len);
    if (om == nullptr) {
        *written = 0;
        return ERROR_INVALID_STATE;
    }
    int rc = ble_gatts_notify_custom(bt->sppConnHandle, nus_tx_handle, om);
    if (rc != 0) {
        *written = 0;
        return ERROR_INVALID_STATE;
    }
    *written = len;
    return ERROR_NONE;
}

static error_t sppRead(struct Device* device, uint8_t* data, size_t max_len, size_t* read_out) {
    auto bt = bt_singleton;
    if (bt == nullptr || data == nullptr || max_len == 0) {
        if (read_out) *read_out = 0;
        return ERROR_NONE;
    }
    auto lock = bt->dataMutex.asScopedLock();
    lock.lock();
    if (bt->sppRxQueue.empty()) {
        *read_out = 0;
        return ERROR_NONE;
    }
    auto& front = bt->sppRxQueue.front();
    size_t copy_len = std::min(front.size(), max_len);
    std::memcpy(data, front.data(), copy_len);
    bt->sppRxQueue.pop_front();
    *read_out = copy_len;
    return ERROR_NONE;
}

static bool sppIsConnected(struct Device* device) {
    auto bt = bt_singleton;
    return bt != nullptr && bt->sppConnHandle != BLE_HS_CONN_HANDLE_NONE;
}

static const BtSerialApi nimble_serial_api = {
    .start = sppStart,
    .stop = sppStop,
    .write = sppWrite,
    .read = sppRead,
    .is_connected = sppIsConnected,
};

// ---- MIDI Active Sensing keepalive ----

static void midiKeepaliveCallback(void* /*arg*/) {
    auto bt = bt_singleton;
    if (bt == nullptr || bt->midiConnHandle == BLE_HS_CONN_HANDLE_NONE) return;
    static const uint8_t as_pkt[3] = { 0x80, 0x80, 0xFE };
    struct os_mbuf* om = ble_hs_mbuf_from_flat(as_pkt, 3);
    if (om == nullptr) return;
    if (bt->midiUseIndicate) {
        ble_gatts_indicate_custom(bt->midiConnHandle, midi_io_handle, om);
    } else {
        ble_gatts_notify_custom(bt->midiConnHandle, midi_io_handle, om);
    }
}

// MIDI (BLE MIDI)
static error_t midiStart(struct Device* device) {
    auto bt = bt_singleton;
    if (bt == nullptr) return ERROR_INVALID_STATE;
    bt->midiActive = true;
    // Create a 2-second periodic Active Sensing timer to prevent Windows BLE
    // MIDI driver from declaring the connection idle and disconnecting.
    if (bt->midiKeepaliveTimer == nullptr) {
        esp_timer_create_args_t args = {};
        args.callback = midiKeepaliveCallback;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name = "midi_as";
        esp_timer_create(&args, &bt->midiKeepaliveTimer);
    }
    esp_timer_start_periodic(bt->midiKeepaliveTimer, 2'000'000); // 2 seconds
    startAdvertising(&MIDI_SVC_UUID);
    return ERROR_NONE;
}

static error_t midiStop(struct Device* device) {
    auto bt = bt_singleton;
    if (bt == nullptr) return ERROR_NONE;
    bt->midiActive = false;
    if (bt->midiKeepaliveTimer != nullptr) {
        esp_timer_stop(bt->midiKeepaliveTimer);
    }
    if (bt->midiConnHandle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(bt->midiConnHandle, BLE_ERR_REM_USER_CONN_TERM);
        bt->midiConnHandle = BLE_HS_CONN_HANDLE_NONE;
    }
    // Do NOT restart advertising — see sppStop for rationale.
    if (!bt->sppActive && !bt->hidActive) {
        ble_gap_adv_stop();
    }
    return ERROR_NONE;
}

static error_t midiSend(struct Device* device, const uint8_t* msg, size_t len) {
    auto bt = bt_singleton;
    if (bt == nullptr || bt->midiConnHandle == BLE_HS_CONN_HANDLE_NONE) {
        return ERROR_INVALID_STATE;
    }
    // BLE MIDI wraps each message with a 2-byte header (timestamp)
    // Header byte 1: 0x80 | (timestamp_high & 0x3F)
    // Header byte 2: 0x80 | (timestamp_low & 0x7F)
    uint8_t timestamp = 0;
    uint8_t header[2] = { static_cast<uint8_t>(0x80 | (timestamp >> 7)), static_cast<uint8_t>(0x80 | (timestamp & 0x7F)) };
    struct os_mbuf* om = ble_hs_mbuf_from_flat(header, 2);
    if (om == nullptr) return ERROR_INVALID_STATE;
    os_mbuf_append(om, msg, len);
    LOGGER.info("midiSend {} bytes (indicate={})", len, bt->midiUseIndicate);
    int rc;
    if (bt->midiUseIndicate) {
        rc = ble_gatts_indicate_custom(bt->midiConnHandle, midi_io_handle, om);
    } else {
        rc = ble_gatts_notify_custom(bt->midiConnHandle, midi_io_handle, om);
    }
    if (rc != 0) {
        LOGGER.error("midiSend failed rc={}", rc);
    }
    return (rc == 0) ? ERROR_NONE : ERROR_INVALID_STATE;
}

static bool midiIsConnected(struct Device* device) {
    auto bt = bt_singleton;
    return bt != nullptr && bt->midiConnHandle != BLE_HS_CONN_HANDLE_NONE;
}

static const BtMidiApi nimble_midi_api = {
    .start = midiStart,
    .stop = midiStop,
    .send = midiSend,
    .is_connected = midiIsConnected,
};

// ---- Service-level SPP/MIDI wrappers (called by C API and external apps) ----

bool sppStart() {
    bool ok = sppStart(nullptr) == ERROR_NONE;
    if (ok) settings::setSppAutoStart(true);
    return ok;
}
void sppStop() {
    sppStop(nullptr);
    settings::setSppAutoStart(false);
}
bool sppWrite(const uint8_t* data, size_t len) {
    size_t written = 0;
    return sppWrite(nullptr, data, len, &written) == ERROR_NONE;
}
size_t sppRead(uint8_t* data, size_t max_len) {
    size_t read_out = 0;
    sppRead(nullptr, data, max_len, &read_out);
    return read_out;
}
bool sppIsConnected() { return sppIsConnected(nullptr); }

bool midiStart() {
    bool ok = midiStart(nullptr) == ERROR_NONE;
    if (ok) settings::setMidiAutoStart(true);
    return ok;
}
void midiStop() {
    midiStop(nullptr);
    settings::setMidiAutoStart(false);
}
bool midiSend(const uint8_t* msg, size_t len) {
    return midiSend(nullptr, msg, len) == ERROR_NONE;
}
bool midiIsConnected() { return midiIsConnected(nullptr); }

size_t midiRead(uint8_t* data, size_t max_len) {
    auto bt = bt_singleton;
    if (bt == nullptr || data == nullptr || max_len == 0) return 0;
    auto lock = bt->dataMutex.asScopedLock();
    lock.lock();
    if (bt->midiRxQueue.empty()) return 0;
    auto& front = bt->midiRxQueue.front();
    size_t copy_len = std::min(front.size(), max_len);
    std::memcpy(data, front.data(), copy_len);
    bt->midiRxQueue.pop_front();
    return copy_len;
}

// ---- HID Device (BLE HID peripheral) ----

static error_t hidDeviceStart(struct Device* /*device*/) {
    auto bt = bt_singleton;
    if (bt == nullptr) return ERROR_INVALID_STATE;
    bt->hidActive = true;
    // switchGattProfile was called by the public hidDeviceStart(uint16_t) before us
    startAdvertisingHid(hid_appearance);
    return ERROR_NONE;
}

static error_t hidDeviceStop(struct Device* /*device*/) {
    auto bt = bt_singleton;
    if (bt == nullptr) return ERROR_NONE;
    bt->hidActive = false;
    if (bt->hidConnHandle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(bt->hidConnHandle, BLE_ERR_REM_USER_CONN_TERM);
        bt->hidConnHandle = BLE_HS_CONN_HANDLE_NONE;
    }
    // Switch back to base (NUS+MIDI only) GATT table so the HID service
    // disappears from GATT and bonded hosts don't try to reconnect as HID.
    if (current_hid_profile != HidProfile::None) {
        switchGattProfile(HidProfile::None);
    }
    return ERROR_NONE;
}

// Send a single key event as an 8-byte boot-protocol keyboard report.
// keycode: USB HID keycode (0x00 = key release).
// Modifier byte is not handled here — use hidSendKeyboard for full control.
static error_t hidDeviceSendKey(struct Device* /*device*/, uint8_t keycode, bool pressed) {
    auto bt = bt_singleton;
    if (bt == nullptr || bt->hidConnHandle == BLE_HS_CONN_HANDLE_NONE) {
        return ERROR_INVALID_STATE;
    }
    uint8_t report[8] = {};
    if (pressed) report[2] = keycode;
    struct os_mbuf* om = ble_hs_mbuf_from_flat(report, sizeof(report));
    if (om == nullptr) return ERROR_INVALID_STATE;
    int rc = ble_gatts_notify_custom(bt->hidConnHandle, hid_kb_input_handle, om);
    return (rc == 0) ? ERROR_NONE : ERROR_INVALID_STATE;
}

static const BtHidApi nimble_hid_api = {
    .host_connect    = nullptr, // HID host: Phase 3
    .host_disconnect = nullptr,
    .device_start    = hidDeviceStart,
    .device_stop     = hidDeviceStop,
    .device_send_key = hidDeviceSendKey,
};

// ---- HID Device service-level send helpers ----
// These expose the full multi-report API (keyboard, consumer, mouse, gamepad)
// at the C++ service layer (similar to midiSend / sppWrite).

static error_t hidSendReport(uint16_t input_handle, const uint8_t* report, size_t len) {
    auto bt = bt_singleton;
    if (bt == nullptr || !bt->hidActive || bt->hidConnHandle == BLE_HS_CONN_HANDLE_NONE) {
        return ERROR_INVALID_STATE;
    }
    struct os_mbuf* om = ble_hs_mbuf_from_flat(report, len);
    if (om == nullptr) return ERROR_INVALID_STATE;
    int rc = ble_gatts_notify_custom(bt->hidConnHandle, input_handle, om);
    if (rc != 0) {
        LOGGER.warn("hidSendReport: notify failed handle={} rc={}", input_handle, rc);
    }
    return (rc == 0) ? ERROR_NONE : ERROR_INVALID_STATE;
}

// Public service-layer wrappers for HID Device

bool hidDeviceStart(uint16_t appearance) {
    hid_appearance = appearance;
    // Map appearance to per-profile GATT table
    HidProfile profile;
    switch (appearance) {
        case 0x03C1: profile = HidProfile::KbConsumer; break;  // Keyboard
        case 0x03C2: profile = HidProfile::Mouse;      break;  // Mouse
        case 0x03C4: profile = HidProfile::Gamepad;    break;  // Gamepad
        default:     profile = HidProfile::KbMouse;    break;  // Generic HID = full Kb+Mouse
    }
    switchGattProfile(profile);
    return hidDeviceStart(nullptr) == ERROR_NONE;
}
void hidDeviceStop()  { hidDeviceStop(nullptr); }
bool hidDeviceIsConnected() {
    auto bt = bt_singleton;
    return bt != nullptr && bt->hidConnHandle != BLE_HS_CONN_HANDLE_NONE;
}

// Send a full 8-byte keyboard input report:
// [0]=modifier [1]=reserved [2..7]=keycodes (up to 6 simultaneous)
bool hidSendKeyboard(const uint8_t report[8]) {
    return hidSendReport(hid_kb_input_handle, report, 8) == ERROR_NONE;
}

// Send a 2-byte consumer/media report: 16-bit HID Consumer usage code (little-endian)
// e.g. {0xE9, 0x00} = Volume Up, {0xEA, 0x00} = Volume Down, {0xCD, 0x00} = Play/Pause
bool hidSendConsumer(const uint8_t report[2]) {
    return hidSendReport(hid_consumer_input_handle, report, 2) == ERROR_NONE;
}

// Send a 4-byte mouse report: [0]=buttons(5bits) [1]=X(rel) [2]=Y(rel) [3]=wheel(rel)
bool hidSendMouse(const uint8_t report[4]) {
    return hidSendReport(hid_mouse_input_handle, report, 4) == ERROR_NONE;
}

// Send an 8-byte gamepad report:
// [0..1]=buttons(16bits) [2]=leftX [3]=leftY [4]=rightX [5]=rightY [6]=L2 [7]=R2
bool hidSendGamepad(const uint8_t report[8]) {
    return hidSendReport(hid_gamepad_input_handle, report, 8) == ERROR_NONE;
}

// ---- HID Host ----
//
// Central-role GATT client that connects to an external BLE HID device
// (keyboard, mouse, gamepad) and subscribes to its Input Report notifications.
// On successful subscription the keyboard reports are forwarded to the LVGL
// hardware keyboard indev so every app receives key events transparently.

static uint32_t hidHostMapKeycode(uint8_t mod, uint8_t kc) {
    bool shift = (mod & 0x22) != 0; // L/R shift bits
    switch (kc) {
        case 0x28: return LV_KEY_ENTER;
        case 0x29: return LV_KEY_ESC;
        case 0x2A: return LV_KEY_BACKSPACE;
        case 0x4C: return LV_KEY_DEL;
        case 0x2B: return shift ? (uint32_t)LV_KEY_PREV : (uint32_t)LV_KEY_NEXT;
        case 0x52: return LV_KEY_UP;
        case 0x51: return LV_KEY_DOWN;
        case 0x50: return LV_KEY_LEFT;
        case 0x4F: return LV_KEY_RIGHT;
        case 0x4A: return LV_KEY_HOME;
        case 0x4D: return LV_KEY_END;
        default: break;
    }
    if (kc >= 0x04 && kc <= 0x1D) {
        uint32_t c = static_cast<uint32_t>('a' + (kc - 0x04));
        return shift ? (c - 0x20) : c;
    }
    if (kc >= 0x1E && kc <= 0x27) {
        static const char nums[]  = "1234567890";
        static const char snums[] = "!@#$%^&*()";
        int i = kc - 0x1E;
        return shift ? static_cast<uint32_t>(snums[i]) : static_cast<uint32_t>(nums[i]);
    }
    if (kc == 0x2C) return ' ';
    return 0;
}

static void hidHostKeyboardReadCb(lv_indev_t* /*indev*/, lv_indev_data_t* data) {
    if (!hid_host_key_queue) { data->state = LV_INDEV_STATE_RELEASED; return; }
    HidHostKeyEvt evt = {};
    if (xQueueReceive(hid_host_key_queue, &evt, 0) == pdTRUE) {
        data->key   = evt.key;
        data->state = evt.pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
        data->continue_reading = (uxQueueMessagesWaiting(hid_host_key_queue) > 0);
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

// Parse a raw HID keyboard input report (8 bytes: mod, reserved, key0..key5).
// Diffs against the previous report to produce proper press and release events,
// matching the same pattern used by the USB HID host module.
static void hidHostHandleKeyboardReport(const uint8_t* data, uint16_t len) {
    if (len < 3 || !hid_host_key_queue) return;
    uint8_t mod = data[0];
    const uint8_t* curr = &data[2];
    int nkeys = std::min((int)(len - 2), 6);

    // Release: keys in prev that are absent in curr
    for (int i = 0; i < 6; i++) {
        uint8_t kc = hid_host_prev_keys[i];
        if (kc == 0) continue;
        bool still = false;
        for (int j = 0; j < nkeys; j++) { if (curr[j] == kc) { still = true; break; } }
        if (!still) {
            uint32_t lv = hidHostMapKeycode(0, kc);
            if (lv) { HidHostKeyEvt e{lv, false}; xQueueSend(hid_host_key_queue, &e, 0); }
        }
    }

    // Press: keys in curr that are absent in prev
    for (int i = 0; i < nkeys; i++) {
        uint8_t kc = curr[i];
        if (kc == 0) continue;
        bool had = false;
        for (int j = 0; j < 6; j++) { if (hid_host_prev_keys[j] == kc) { had = true; break; } }
        if (!had) {
            uint32_t lv = hidHostMapKeycode(mod, kc);
            if (lv) { HidHostKeyEvt e{lv, true}; xQueueSend(hid_host_key_queue, &e, 0); }
        }
    }

    std::memcpy(hid_host_prev_keys, curr, 6);
}

// Mouse read callback for LVGL pointer indev — called from LVGL task.
// Applies the inverse of LVGL's rotation transform so the cursor tracks correctly.
static void hidHostMouseReadCb(lv_indev_t* /*indev*/, lv_indev_data_t* data) {
    int32_t cx = hid_host_mouse_x.load();
    int32_t cy = hid_host_mouse_y.load();

    lv_display_t* disp = lv_display_get_default();
    if (disp) {
        int32_t ow = lv_display_get_original_horizontal_resolution(disp);
        int32_t oh = lv_display_get_original_vertical_resolution(disp);
        switch (lv_display_get_rotation(disp)) {
            case LV_DISPLAY_ROTATION_0:
                data->point.x = (lv_coord_t)cx;
                data->point.y = (lv_coord_t)cy;
                break;
            case LV_DISPLAY_ROTATION_90:
                data->point.x = (lv_coord_t)cy;
                data->point.y = (lv_coord_t)(oh - cx - 1);
                break;
            case LV_DISPLAY_ROTATION_180:
                data->point.x = (lv_coord_t)(ow - cx - 1);
                data->point.y = (lv_coord_t)(oh - cy - 1);
                break;
            case LV_DISPLAY_ROTATION_270:
                data->point.x = (lv_coord_t)(ow - cy - 1);
                data->point.y = (lv_coord_t)cx;
                break;
        }
    } else {
        data->point.x = (lv_coord_t)cx;
        data->point.y = (lv_coord_t)cy;
    }
    data->state = hid_host_mouse_btn.load() ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;

    // Show cursor on first actual mouse activity
    if (!hid_host_mouse_active.load()) {
        hid_host_mouse_active.store(true);
        if (hid_host_ctx && hid_host_ctx->mouseCursor) {
            lv_obj_remove_flag(hid_host_ctx->mouseCursor, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

// Parse a raw HID mouse input report: [buttons, dx, dy, (wheel)].
// Updates the global atomic mouse position clamped to display bounds.
static void hidHostHandleMouseReport(const uint8_t* data, uint16_t len) {
    if (len < 3) return;
    bool btn      = (data[0] & 0x01) != 0;
    int8_t dx     = (int8_t)data[1];
    int8_t dy     = (int8_t)data[2];

    lv_display_t* disp = lv_display_get_default();
    int32_t w = disp ? lv_display_get_horizontal_resolution(disp) : 320;
    int32_t h = disp ? lv_display_get_vertical_resolution(disp)   : 240;

    int32_t nx = hid_host_mouse_x.load() + dx;
    int32_t ny = hid_host_mouse_y.load() + dy;
    if (nx < 0) nx = 0;
    if (nx >= w) nx = w - 1;
    if (ny < 0) ny = 0;
    if (ny >= h) ny = h - 1;

    hid_host_mouse_x.store(nx);
    hid_host_mouse_y.store(ny);
    hid_host_mouse_btn.store(btn);

    // Lazily create mouse indev on the first mouse report from this device
    if (hid_host_ctx && hid_host_ctx->mouseIndev == nullptr) {
        getMainDispatcher().dispatch([] {
            if (!hid_host_ctx || hid_host_ctx->mouseIndev != nullptr) return;
            if (!tt::lvgl::lock(1000)) { LOGGER.warn("HID host: LVGL lock failed for mouse indev"); return; }
            auto* ms = lv_indev_create();
            lv_indev_set_type(ms, LV_INDEV_TYPE_POINTER);
            lv_indev_set_read_cb(ms, hidHostMouseReadCb);
            auto* cur = lv_image_create(lv_layer_sys());
            lv_obj_remove_flag(cur, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_flag(cur, LV_OBJ_FLAG_HIDDEN); // shown on first actual movement
            lv_image_set_src(cur, TT_ASSETS_UI_CURSOR);
            lv_indev_set_cursor(ms, cur);
            hid_host_ctx->mouseIndev  = ms;
            hid_host_ctx->mouseCursor = cur;
            tt::lvgl::unlock();
            LOGGER.info("HID host: mouse indev registered");
        });
    }
}

// Subscribe to the next pending Input Report CCCD
static void hidHostSubscribeNext(HidHostCtx& ctx);

// Timer callback: fires 500ms after ENC_CHANGE to let the device settle before we write CCCDs.
// Called from the esp_timer task — safe for NimBLE GATT calls (same as advRestartCallback).
static void hidEncRetryTimerCb(void* /*arg*/) {
    if (hid_host_ctx) {
        LOGGER.info("HID host: post-encryption delay complete — starting CCCD subscriptions");
        hidHostSubscribeNext(*hid_host_ctx);
    }
}

static int hidHostCccdWriteCb(uint16_t conn_handle, const struct ble_gatt_error* error,
                               struct ble_gatt_attr* /*attr*/, void* /*arg*/) {
    if (!hid_host_ctx) return 0;
    auto& ctx = *hid_host_ctx;
    if (conn_handle != ctx.connHandle) return 0;

    if (error->status != 0 && error->status != BLE_HS_EDONE) {
        // BLE_ATT_ERR_INSUFFICIENT_AUTHEN (0x05) and BLE_ATT_ERR_INSUFFICIENT_ENC (0x0F)
        // mean the device requires an encrypted link — initiate pairing and retry after ENC_CHANGE.
        if ((error->status == BLE_HS_ATT_ERR(BLE_ATT_ERR_INSUFFICIENT_AUTHEN) ||
             error->status == BLE_HS_ATT_ERR(BLE_ATT_ERR_INSUFFICIENT_ENC))
            && !ctx.securityInitiated) {
            LOGGER.info("HID host: CCCD auth required — initiating security");
            ctx.securityInitiated = true;
            ble_gap_security_initiate(conn_handle);
            // Do NOT advance subscribeIdx; retry will restart from subscribeIdx=0 on ENC_CHANGE.
            return 0;
        }
        // GATT timeout: the device didn't respond to this CCCD write in 30s.
        // On bonded reconnect, some devices don't respond (CCCD already set from prior session).
        // Skip this report and continue — the device may still send notifications.
        if (error->status == BLE_HS_ETIMEOUT) {
            LOGGER.warn("HID host: CCCD write timed out for report[{}] — skipping", ctx.subscribeIdx);
            ctx.subscribeIdx++;
            hidHostSubscribeNext(ctx);
            return 0;
        }
        if (error->status == BLE_HS_ENOTCONN) {
            LOGGER.warn("HID host: CCCD write failed — not connected");
            return 0;
        }
        LOGGER.warn("HID host: CCCD write failed status={}", error->status);
    }
    ctx.subscribeIdx++;
    hidHostSubscribeNext(ctx);
    return 0;
}

static void hidHostSubscribeNext(HidHostCtx& ctx) {
    if (ctx.subscribeIdx >= (int)ctx.inputRpts.size()) {
        // Guard: ENC_CHANGE starts a 500ms timer; if that timer fires after the CCCD write
        // already completed and reached the ready block, we'd run this block twice.
        if (ctx.readyBlockFired) {
            LOGGER.info("HID host: subscribe ready block already ran — ignoring duplicate call");
            return;
        }
        ctx.readyBlockFired = true;
        LOGGER.info("HID host: all {} reports subscribed — ready", ctx.inputRpts.size());
        // Stop the retry timer (it may still be pending if CCCD write completed before 500ms delay)
        if (hid_enc_retry_timer) {
            esp_timer_stop(hid_enc_retry_timer);
        }
        // Create key queue and keyboard indev eagerly so the keyboard is available
        // as soon as connection is ready (avoids waiting for first keypress).
        if (!hid_host_key_queue) {
            hid_host_key_queue = xQueueCreate(HID_HOST_KEY_QUEUE_SIZE, sizeof(HidHostKeyEvt));
        }
        auto bt_for_kb = ctx.bt;
        getMainDispatcher().dispatch([bt_for_kb] {
            if (!hid_host_ctx || hid_host_ctx->kbIndev != nullptr) return;
            if (!tt::lvgl::lock(1000)) { LOGGER.warn("HID host: LVGL lock failed for kb indev"); return; }
            auto* kb = lv_indev_create();
            lv_indev_set_type(kb, LV_INDEV_TYPE_KEYPAD);
            lv_indev_set_read_cb(kb, hidHostKeyboardReadCb);
            hid_host_ctx->kbIndev = kb;
            tt::lvgl::hardware_keyboard_set_indev(kb);
            tt::lvgl::unlock();
            LOGGER.info("HID host: keyboard indev registered");
        });
        // Save to paired store so the device appears in BtManage "Paired" section.
        // Dispatch to main task since file I/O shouldn't run on the NimBLE task.
        // Note: save even if we disconnect before the dispatch runs — device should be remembered.
        auto peer_addr = ctx.peerAddr;
        auto bt_weak = ctx.bt;
        getMainDispatcher().dispatch([peer_addr, bt_weak] {
            // Find device name from scan results
            std::string name;
            if (auto bt = bt_weak) {
                auto lock = bt->dataMutex.asScopedLock();
                lock.lock();
                for (const auto& r : bt->scanResults) {
                    if (r.addr == peer_addr) { name = r.name; break; }
                }
            }
            settings::PairedDevice device;
            device.addr        = peer_addr;
            device.name        = name;
            device.profileId   = BT_PROFILE_HID_HOST;
            device.autoConnect = true; // reconnect automatically when device is seen during scan
            settings::save(device);
            // Publish AFTER save so BtManage sees the file when it calls updatePairedPeers()
            publishEvent(bt_weak, BtEvent::ProfileStateChanged);
        });
        return;
    }
    auto& rpt = ctx.inputRpts[ctx.subscribeIdx];
    if (rpt.cccdHandle == 0) {
        // No CCCD discovered — skip
        ctx.subscribeIdx++;
        hidHostSubscribeNext(ctx);
        return;
    }
    static const uint16_t notify_val = 0x0001;
    int rc = ble_gattc_write_flat(ctx.connHandle, rpt.cccdHandle,
                                   &notify_val, sizeof(notify_val),
                                   hidHostCccdWriteCb, nullptr);
    if (rc != 0) {
        LOGGER.warn("HID host: gattc_write_flat CCCD failed rc={}", rc);
        ctx.subscribeIdx++;
        hidHostSubscribeNext(ctx);
    }
}

// Descriptor discovery callback: find CCCD (0x2902) handle for each Input Report char
static int hidHostDscDiscCb(uint16_t conn_handle, const struct ble_gatt_error* error,
                             uint16_t chr_val_handle, const struct ble_gatt_dsc* dsc, void* arg) {
    if (!hid_host_ctx) return 0;
    auto& ctx = *hid_host_ctx;
    if (conn_handle != ctx.connHandle) return 0;

    if (error->status == 0 && dsc != nullptr) {
        uint16_t dsc_uuid = ble_uuid_u16(&dsc->uuid.u);
        if (dsc_uuid == 0x2902) {
            // Found CCCD for this char
            for (auto& rpt : ctx.inputRpts) {
                if (rpt.valHandle == chr_val_handle) {
                    rpt.cccdHandle = dsc->handle;
                    LOGGER.info("HID host: CCCD handle={} for val_handle={}", dsc->handle, chr_val_handle);
                    break;
                }
            }
        }
    } else if (error->status == BLE_HS_EDONE) {
        // Move to next input report's descriptor discovery
        auto* idx_ptr = static_cast<int*>(arg);
        int next_idx = (*idx_ptr) + 1;

        if (next_idx < (int)ctx.inputRpts.size()) {
            static int dsc_idx;
            dsc_idx = next_idx;
            auto& next_rpt = ctx.inputRpts[next_idx];
            // End handle = next char's val handle - 1, or svc end handle
            uint16_t end = (next_idx + 1 < (int)ctx.inputRpts.size())
                           ? ctx.inputRpts[next_idx + 1].valHandle - 1
                           : ctx.hidSvcEnd;
            int rc = ble_gattc_disc_all_dscs(ctx.connHandle, next_rpt.valHandle, end,
                                              hidHostDscDiscCb, &dsc_idx);
            if (rc != 0) {
                LOGGER.warn("HID host: disc_all_dscs[{}] failed rc={}", next_idx, rc);
                ctx.subscribeIdx = 0;
                hidHostSubscribeNext(ctx);
            }
        } else {
            // All descriptor discovery done — start subscribing
            ctx.subscribeIdx = 0;
            hidHostSubscribeNext(ctx);
        }
    }
    return 0;
}

// Characteristic discovery callback: collect Input Report chars (UUID 0x2A4D, NOTIFY)
static int hidHostChrDiscCb(uint16_t conn_handle, const struct ble_gatt_error* error,
                             const struct ble_gatt_chr* chr, void* arg) {
    if (!hid_host_ctx) return 0;
    auto& ctx = *hid_host_ctx;
    if (conn_handle != ctx.connHandle) return 0;

    if (error->status == 0 && chr != nullptr) {
        uint16_t uuid16 = ble_uuid_u16(&chr->uuid.u);
        if (uuid16 == 0x2A4D && (chr->properties & BLE_GATT_CHR_PROP_NOTIFY)) {
            HidHostInputRpt rpt = {};
            rpt.valHandle  = chr->val_handle;
            rpt.cccdHandle = 0;
            rpt.reportId   = 0;
            ctx.inputRpts.push_back(rpt);
            LOGGER.info("HID host: Input Report chr val_handle={}", chr->val_handle);
        }
    } else if (error->status == BLE_HS_EDONE) {
        if (ctx.inputRpts.empty()) {
            LOGGER.warn("HID host: no Input Report chars — disconnecting");
            ble_gap_terminate(ctx.connHandle, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }
        // Discover descriptors for first input report
        static int dsc_idx = 0;
        dsc_idx = 0;
        auto& first = ctx.inputRpts[0];
        uint16_t end = (ctx.inputRpts.size() > 1)
                       ? ctx.inputRpts[1].valHandle - 1
                       : ctx.hidSvcEnd;
        int rc = ble_gattc_disc_all_dscs(ctx.connHandle, first.valHandle, end,
                                          hidHostDscDiscCb, &dsc_idx);
        if (rc != 0) {
            LOGGER.warn("HID host: disc_all_dscs[0] failed rc={}", rc);
            ctx.subscribeIdx = 0;
            hidHostSubscribeNext(ctx);
        }
    }
    return 0;
}

// Service discovery callback: find the HID service (UUID 0x1812)
static int hidHostSvcDiscCb(uint16_t conn_handle, const struct ble_gatt_error* error,
                             const struct ble_gatt_svc* svc, void* arg) {
    if (!hid_host_ctx) return 0;
    auto& ctx = *hid_host_ctx;
    if (conn_handle != ctx.connHandle) return 0;

    if (error->status == 0 && svc != nullptr) {
        if (ble_uuid_u16(&svc->uuid.u) == 0x1812) {
            ctx.hidSvcStart = svc->start_handle;
            ctx.hidSvcEnd   = svc->end_handle;
            LOGGER.info("HID host: HID service start={} end={}", ctx.hidSvcStart, ctx.hidSvcEnd);
        }
    } else if (error->status == BLE_HS_EDONE) {
        if (ctx.hidSvcStart == 0) {
            LOGGER.warn("HID host: no HID service found — disconnecting");
            ble_gap_terminate(ctx.connHandle, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }
        int rc = ble_gattc_disc_all_chrs(ctx.connHandle, ctx.hidSvcStart, ctx.hidSvcEnd,
                                          hidHostChrDiscCb, nullptr);
        if (rc != 0) {
            LOGGER.warn("HID host: disc_all_chrs failed rc={}", rc);
            ble_gap_terminate(ctx.connHandle, BLE_ERR_REM_USER_CONN_TERM);
        }
    }
    return 0;
}

// GAP event handler for the central (HID host) connection
static int hidHostGapCb(struct ble_gap_event* event, void* arg) {
    if (!hid_host_ctx) return 0;
    auto& ctx = *hid_host_ctx;

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ctx.connHandle = event->connect.conn_handle;
                LOGGER.info("HID host: connected (handle={})", ctx.connHandle);
                int rc = ble_gattc_disc_all_svcs(ctx.connHandle, hidHostSvcDiscCb, nullptr);
                if (rc != 0) {
                    LOGGER.warn("HID host: disc_all_svcs failed rc={}", rc);
                    ble_gap_terminate(ctx.connHandle, BLE_ERR_REM_USER_CONN_TERM);
                }
            } else {
                LOGGER.warn("HID host: connect failed status={}", event->connect.status);
                if (hid_host_ctx) publishEvent(hid_host_ctx->bt, BtEvent::ProfileStateChanged);
                hid_host_ctx.reset();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            LOGGER.info("HID host: disconnected reason={}", event->disconnect.reason);
            {
                auto bt = ctx.bt;
                // Move ctx out immediately so hidHostConnect() can be called right away
                // (before the main-task LVGL cleanup dispatch runs).
                // Extract LVGL object pointers first — unique_ptr is not copy-constructible
                // so we can't capture it in a std::function lambda.
                lv_indev_t* saved_kb     = hid_host_ctx ? hid_host_ctx->kbIndev     : nullptr;
                lv_indev_t* saved_mouse  = hid_host_ctx ? hid_host_ctx->mouseIndev  : nullptr;
                lv_obj_t*   saved_cursor = hid_host_ctx ? hid_host_ctx->mouseCursor : nullptr;
                hid_host_ctx.reset();
                if (hid_host_key_queue) {
                    vQueueDelete(hid_host_key_queue);
                    hid_host_key_queue = nullptr;
                }
                std::memset(hid_host_prev_keys, 0, sizeof(hid_host_prev_keys));
                hid_host_mouse_x.store(0);
                hid_host_mouse_y.store(0);
                hid_host_mouse_btn.store(false);
                hid_host_mouse_active.store(false);
                // Dispatch only the LVGL object cleanup to the main task
                getMainDispatcher().dispatch([saved_kb, saved_mouse, saved_cursor] {
                    if (!tt::lvgl::lock(1000)) {
                        LOGGER.warn("HID host: failed to acquire LVGL lock for indev cleanup");
                        return;
                    }
                    if (saved_kb) {
                        tt::lvgl::hardware_keyboard_set_indev(nullptr);
                        lv_indev_delete(saved_kb);
                    }
                    if (saved_mouse)  lv_indev_delete(saved_mouse);
                    if (saved_cursor) lv_obj_delete(saved_cursor);
                    tt::lvgl::unlock();
                });
                publishEvent(bt, BtEvent::ProfileStateChanged);
            }
            break;

        case BLE_GAP_EVENT_ENC_CHANGE:
            if (event->enc_change.conn_handle == ctx.connHandle) {
                if (event->enc_change.status == 0) {
                    LOGGER.info("HID host: encryption established — retrying CCCD subscriptions in 500ms");
                    ctx.subscribeIdx = 0;
                    if (hid_enc_retry_timer) {
                        esp_timer_stop(hid_enc_retry_timer); // cancel if already running
                        esp_timer_start_once(hid_enc_retry_timer, 500 * 1000);
                    } else {
                        hidHostSubscribeNext(ctx);
                    }
                } else {
                    LOGGER.warn("HID host: encryption failed status={}", event->enc_change.status);
                }
            }
            break;

        case BLE_GAP_EVENT_NOTIFY_RX:
            if (event->notify_rx.conn_handle == ctx.connHandle) {
                uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
                if (len > 0 && len <= 32) {
                    uint8_t buf[32] = {};
                    os_mbuf_copydata(event->notify_rx.om, 0, len, buf);
                    for (const auto& rpt : ctx.inputRpts) {
                        if (rpt.valHandle == event->notify_rx.attr_handle) {
                            if (len >= 6) {
                                hidHostHandleKeyboardReport(buf, len);
                            } else if (len >= 3) {
                                hidHostHandleMouseReport(buf, len);
                            }
                            break;
                        }
                    }
                }
            }
            break;

        default:
            break;
    }
    return 0;
}

void hidHostConnect(const std::array<uint8_t, 6>& addr) {
    auto bt = bt_singleton;
    if (bt == nullptr) return;
    if (bt->getRadioState() != RadioState::On) {
        LOGGER.warn("hidHostConnect: radio not on");
        return;
    }
    if (hid_host_ctx) {
        LOGGER.warn("hidHostConnect: already connecting/connected");
        return;
    }

    // Reset mouse position for fresh connection
    hid_host_mouse_x.store(0);
    hid_host_mouse_y.store(0);
    hid_host_mouse_btn.store(false);
    hid_host_mouse_active.store(false);

    hid_host_ctx = std::make_unique<HidHostCtx>();
    hid_host_ctx->bt = bt;
    hid_host_ctx->peerAddr = addr;

    // Look up the address type from the most recent scan results so random-address
    // devices (type=1) are connected correctly. Fall back to PUBLIC if not found.
    ble_addr_t ble_addr = {};
    ble_addr.type = BLE_ADDR_PUBLIC;
    std::memcpy(ble_addr.val, addr.data(), 6);
    {
        auto lock = bt->dataMutex.asScopedLock();
        for (const auto& sa : bt->scanAddresses) {
            if (std::memcmp(sa.val, addr.data(), 6) == 0) {
                ble_addr.type = sa.type;
                break;
            }
        }
    }

    uint8_t own_addr_type;
    ble_hs_id_infer_auto(0, &own_addr_type);

    int rc = ble_gap_connect(own_addr_type, &ble_addr, 5000, nullptr, hidHostGapCb, nullptr);
    if (rc != 0) {
        LOGGER.warn("hidHostConnect: ble_gap_connect failed rc={}", rc);
        hid_host_ctx.reset();
    } else {
        LOGGER.info("hidHostConnect: connecting...");
    }
}

void hidHostDisconnect() {
    if (!hid_host_ctx || hid_host_ctx->connHandle == BLE_HS_CONN_HANDLE_NONE) return;
    ble_gap_terminate(hid_host_ctx->connHandle, BLE_ERR_REM_USER_CONN_TERM);
}

static bool hidHostIsConnectedImpl() {
    return hid_host_ctx != nullptr &&
           hid_host_ctx->connHandle != BLE_HS_CONN_HANDLE_NONE &&
           !hid_host_ctx->inputRpts.empty() &&
           hid_host_ctx->subscribeIdx >= (int)hid_host_ctx->inputRpts.size();
}

// ---- BluetoothApi instance ----

const BluetoothApi nimble_bluetooth_api = {
    // Core radio functions not used directly (service talks to NimBLE directly)
    // These are here for HAL device driver use
    .get_radio_state = nullptr,
    .set_radio_enabled = nullptr,
    .scan_start = nullptr,
    .scan_stop = nullptr,
    .is_scanning = nullptr,
    .pair = nullptr,
    .unpair = nullptr,
    .get_paired_peers = nullptr,
    .connect = nullptr,
    .disconnect = nullptr,
    .add_event_callback = nullptr,
    .remove_event_callback = nullptr,
    .hid = &nimble_hid_api,
    .serial = &nimble_serial_api,
    .midi = &nimble_midi_api,
};

// ---- BLUETOOTH_TYPE device type ----

const struct DeviceType BLUETOOTH_TYPE = {
    .name = "bluetooth",
};

// ---- Service ----

class BluetoothService final : public Service {

public:

    bool onStart(ServiceContext& service) override {
        assert(bt_singleton == nullptr);
        bt_singleton = std::make_shared<Bluetooth>();

        esp_timer_create_args_t enc_args = {};
        enc_args.callback        = hidEncRetryTimerCb;
        enc_args.dispatch_method = ESP_TIMER_TASK;
        enc_args.name            = "hid_enc_retry";
        esp_timer_create(&enc_args, &hid_enc_retry_timer);

        if (settings::shouldEnableOnBoot()) {
            LOGGER.info("Auto-enabling Bluetooth on boot");
            auto bt = bt_singleton;
            getMainDispatcher().dispatch([bt] { dispatchEnable(bt); });
        }

        return true;
    }

    void onStop(ServiceContext& service) override {
        auto bt = bt_singleton;
        assert(bt != nullptr);

        if (hid_enc_retry_timer) {
            esp_timer_stop(hid_enc_retry_timer);
            esp_timer_delete(hid_enc_retry_timer);
            hid_enc_retry_timer = nullptr;
        }

        if (bt->getRadioState() != RadioState::Off) {
            dispatchDisable(bt);
        }

        auto lock_data = bt->dataMutex.asScopedLock();
        lock_data.lock();
        auto lock_radio = bt->radioMutex.asScopedLock();
        lock_radio.lock();

        bt_singleton = nullptr;
    }
};

extern const ServiceManifest manifest = {
    .id = "bluetooth",
    .createService = create<BluetoothService>
};

} // namespace tt::service::bluetooth

#endif // CONFIG_BT_NIMBLE_ENABLED
