#ifdef ESP_PLATFORM
#include <sdkconfig.h>
#endif

#if defined(CONFIG_BT_NIMBLE_ENABLED)

#include <Tactility/service/bluetooth/BluetoothNimBLEInternal.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

namespace tt::service::bluetooth {

static const auto LOGGER = Logger("BtService");

HidProfile current_hid_profile = HidProfile::None;

// appearance stored so advRestartCallback can re-use it without an extra parameter
uint16_t hid_appearance = 0x03C1;

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
const uint8_t* active_hid_rpt_map    = nullptr;
size_t         active_hid_rpt_map_len = 0;

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
uint16_t hid_kb_input_handle;
uint16_t hid_consumer_input_handle;
uint16_t hid_mouse_input_handle;
uint16_t hid_gamepad_input_handle;

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

// ---- GATT profile switch ----

// Rebuild the GATT table for the selected HID profile.
// Must only be called from the NimBLE host task (e.g., from hidDeviceStart/Stop
// which run on the main dispatcher). Terminates any active HID connection,
// resets all GATT tables, then re-registers NUS + MIDI + selected HID profile.
void switchGattProfile(HidProfile profile) {
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

void hidDeviceInitGatt() {
    current_hid_profile    = HidProfile::None;
    active_hid_rpt_map     = nullptr;
    active_hid_rpt_map_len = 0;
    int rc = ble_gatts_count_cfg(gatt_svcs_none);
    if (rc != 0) {
        LOGGER.error("gatts_count_cfg failed (rc={})", rc);
    } else {
        rc = ble_gatts_add_svcs(gatt_svcs_none);
        if (rc != 0) {
            LOGGER.error("gatts_add_svcs failed (rc={})", rc);
        }
    }
}

void hidDeviceInitGattHandles() {
    // HID handles are populated by NimBLE via the val_handle pointers in the char arrays.
    // Nothing to do here at init time since switchGattProfile handles re-registration.
    // This function exists so core can call it after ble_gatts_add_svcs for symmetry
    // with sppInitGattHandles / midiInitGattHandles.
    (void)hid_kb_input_handle;
    (void)hid_consumer_input_handle;
    (void)hid_mouse_input_handle;
    (void)hid_gamepad_input_handle;
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

const BtHidApi nimble_hid_api = {
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

} // namespace tt::service::bluetooth

#pragma GCC diagnostic pop

#endif // CONFIG_BT_NIMBLE_ENABLED
