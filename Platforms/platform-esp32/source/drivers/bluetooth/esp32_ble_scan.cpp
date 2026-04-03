#ifdef ESP_PLATFORM
#include <sdkconfig.h>
#endif

#if defined(CONFIG_BT_NIMBLE_ENABLED)

#include <bluetooth/esp32_ble_internal.h>

#include <host/ble_gap.h>
#include <host/ble_hs_mbuf.h>

#include <algorithm>
#include <cstring>

#define TAG "esp32_ble_scan"
#include <tactility/log.h>

// ---- Module-static scan data ----
// Scan results and their associated state are owned entirely by this module.
// esp32_ble.cpp clears them via ble_scan_clear_results() before each new scan.

#define BLE_SCAN_MAX 64

static SemaphoreHandle_t s_scan_mutex   = nullptr;
static BtPeerRecord      s_scan_results[BLE_SCAN_MAX];
static ble_addr_t        s_scan_addrs[BLE_SCAN_MAX]; // full ble_addr_t (type+val) for connections
static size_t            s_scan_count   = 0;

// Module-static device pointer used only by the name-resolution GAP/GATT callbacks
// whose void* arg is already occupied by the peer index (uintptr_t).
// Set at the start of ble_resolve_next_unnamed_peer; valid for the duration of
// the sequential resolution chain (single-device, single-scan at a time).
static struct Device* s_scan_device = nullptr;

// ---- Scan data lifecycle ----

void ble_scan_init() {
    s_scan_mutex = xSemaphoreCreateMutex();
    s_scan_count = 0;
    memset(s_scan_results, 0, sizeof(s_scan_results));
}

void ble_scan_deinit() {
    if (s_scan_mutex != nullptr) {
        vSemaphoreDelete(s_scan_mutex);
        s_scan_mutex = nullptr;
    }
    s_scan_count = 0;
}

void ble_scan_clear_results() {
    xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    s_scan_count = 0;
    memset(s_scan_results, 0, sizeof(s_scan_results));
    xSemaphoreGive(s_scan_mutex);
}

// ---- GAP scan callback ----

int ble_gap_disc_event_handler(struct ble_gap_event* event, void* arg) {
    struct Device* device = (struct Device*)arg;

    switch (event->type) {
        case BLE_GAP_EVENT_DISC: {
            const auto& disc = event->disc;

            BtPeerRecord record = {};
            memcpy(record.addr, disc.addr.val, BT_ADDR_LEN);
            record.addr_type = disc.addr.type;
            record.rssi      = disc.rssi;
            record.paired    = false;
            record.connected = false;

            struct ble_hs_adv_fields fields;
            if (ble_hs_adv_parse_fields(&fields, disc.data, disc.length_data) == 0) {
                if (fields.name != nullptr && fields.name_len > 0) {
                    size_t copy_len = std::min<size_t>(fields.name_len, BT_NAME_MAX);
                    memcpy(record.name, fields.name, copy_len);
                    record.name[copy_len] = '\0';
                }
            }

            {
                xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
                bool found = false;
                for (size_t i = 0; i < s_scan_count; ++i) {
                    if (memcmp(s_scan_results[i].addr, record.addr, BT_ADDR_LEN) == 0) {
                        // Deduplicate: merge name from SCAN_RSP without clobbering ADV_IND name
                        if (record.name[0] != '\0') {
                            memcpy(s_scan_results[i].name, record.name, BT_NAME_MAX + 1);
                        }
                        s_scan_results[i].rssi = record.rssi;
                        found = true;
                        break;
                    }
                }
                if (!found && s_scan_count < BLE_SCAN_MAX) {
                    s_scan_results[s_scan_count] = record;
                    s_scan_addrs[s_scan_count]   = disc.addr; // full addr (type+val)
                    s_scan_count++;
                }
                xSemaphoreGive(s_scan_mutex);
            }

            struct BtEvent e = {};
            e.type = BT_EVENT_PEER_FOUND;
            e.peer  = record;
            ble_publish_event(device, e);
            break;
        }

        case BLE_GAP_EVENT_DISC_COMPLETE:
            LOG_I(TAG, "Scan complete (reason=%d)", event->disc_complete.reason);
            // Keep scan_active=true; resolveNextUnnamedPeer clears it and fires ScanFinished
            // once name resolution finishes, so the UI spinner stays active throughout.
            ble_resolve_next_unnamed_peer(device, 0);
            break;

        default:
            break;
    }
    return 0;
}

// ---- GATT Device Name resolution ----
//
// After a scan completes, briefly connect to each device that didn't include its
// name in advertising data and read Generic Access Device Name (UUID 0x2A00).
//
// Resolution is sequential: connect → read → disconnect → next device.
// Skip resolution if a profile server or HID host connection is active —
// a simultaneous central connection would fail with BLE_HS_EALREADY.

static int name_read_callback(uint16_t conn_handle, const struct ble_gatt_error* error,
                              struct ble_gatt_attr* attr, void* arg) {
    struct Device* device = s_scan_device;

    if (error->status == 0 && attr != nullptr) {
        size_t idx = (size_t)(uintptr_t)arg;
        uint16_t len = OS_MBUF_PKTLEN(attr->om);
        if (len > 0 && len <= (uint16_t)BT_NAME_MAX) {
            char name_buf[BT_NAME_MAX + 1] = {};
            os_mbuf_copydata(attr->om, 0, len, name_buf);
            {
                xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
                if (idx < s_scan_count && s_scan_results[idx].name[0] == '\0') {
                    memcpy(s_scan_results[idx].name, name_buf, len);
                    s_scan_results[idx].name[len] = '\0';
                    LOG_I(TAG, "Name resolved (idx=%u): %s", (unsigned)idx, name_buf);
                }
                BtPeerRecord record = (idx < s_scan_count) ? s_scan_results[idx] : BtPeerRecord{};
                xSemaphoreGive(s_scan_mutex);

                struct BtEvent e = {};
                e.type = BT_EVENT_PEER_FOUND;
                e.peer  = record;
                ble_publish_event(device, e);
            }
        }
        return 0; // wait for BLE_HS_EDONE
    }

    // BLE_HS_EDONE, ATT error, or timeout — done with this device
    ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    return 0;
}

static int name_res_gap_callback(struct ble_gap_event* event, void* arg) {
    size_t idx = (size_t)(uintptr_t)arg;
    struct Device* device = s_scan_device;

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                LOG_I(TAG, "Name resolution: connected (idx=%u handle=%u)", (unsigned)idx, event->connect.conn_handle);
                static const ble_uuid16_t device_name_uuid = BLE_UUID16_INIT(0x2A00);
                int rc = ble_gattc_read_by_uuid(event->connect.conn_handle,
                                                1, 0xFFFF,
                                                &device_name_uuid.u,
                                                name_read_callback, arg);
                if (rc != 0) {
                    LOG_W(TAG, "Name resolution: read_by_uuid failed rc=%d", rc);
                    ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                }
            } else {
                LOG_I(TAG, "Name resolution: connect failed (idx=%u status=%d)", (unsigned)idx, event->connect.status);
                ble_resolve_next_unnamed_peer(device, idx + 1);
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            LOG_I(TAG, "Name resolution: disconnected (idx=%u)", (unsigned)idx);
            ble_resolve_next_unnamed_peer(device, idx + 1);
            break;

        default:
            break;
    }
    return 0;
}

void ble_resolve_next_unnamed_peer(struct Device* device, size_t start_idx) {
    s_scan_device = device;

    // Skip if a profile server or HID host connection attempt is active —
    // initiating a central connection simultaneously would fail (BLE_HS_EALREADY).
    if (ble_ctx_get_midi_active(device) || ble_ctx_get_spp_active(device) ||
        ble_ctx_get_hid_active(device)  || ble_ctx_get_hid_host_active(device)) {
        LOG_I(TAG, "Name resolution: skipping (server or HID host active)");
        ble_ctx_set_scan_active(device, false);
        struct BtEvent e = {};
        e.type = BT_EVENT_SCAN_FINISHED;
        ble_publish_event(device, e);
        return;
    }

    size_t i = start_idx;
    while (true) {
        ble_addr_t addr  = {};
        bool       found = false;
        {
            xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
            while (i < s_scan_count) {
                if (s_scan_results[i].name[0] == '\0') {
                    addr  = s_scan_addrs[i];
                    found = true;
                    break;
                }
                ++i;
            }
            xSemaphoreGive(s_scan_mutex);
        }

        if (!found) {
            LOG_I(TAG, "Name resolution: complete (%u devices)", (unsigned)i);
            ble_ctx_set_scan_active(device, false);
            struct BtEvent e = {};
            e.type = BT_EVENT_SCAN_FINISHED;
            ble_publish_event(device, e);
            return;
        }

        uint8_t own_addr_type;
        ble_hs_id_infer_auto(0, &own_addr_type);

        void* idx_arg = (void*)(uintptr_t)i;
        int rc = ble_gap_connect(own_addr_type, &addr, 1500, nullptr,
                                 name_res_gap_callback, idx_arg);
        if (rc == 0) {
            return; // name_res_gap_callback continues the chain
        }

        LOG_I(TAG, "Name resolution: ble_gap_connect failed idx=%u rc=%d, skipping", (unsigned)i, rc);
        ++i;
    }
}

#endif // CONFIG_BT_NIMBLE_ENABLED
