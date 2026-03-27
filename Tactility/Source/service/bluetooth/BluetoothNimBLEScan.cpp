#ifdef ESP_PLATFORM
#include <sdkconfig.h>
#endif

#if defined(CONFIG_BT_NIMBLE_ENABLED)

#include <Tactility/service/bluetooth/BluetoothNimBLEInternal.h>

namespace tt::service::bluetooth {

static const auto LOGGER = Logger("BtService");

// ---- GAP scan callback ----

int gapDiscEventHandler(struct ble_gap_event* event, void* arg) {
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
void dispatchAutoConnectHidHost(std::shared_ptr<Bluetooth> bt) {
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

void resolveNextUnnamedPeer(std::shared_ptr<Bluetooth> bt, size_t start_idx) {
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

} // namespace tt::service::bluetooth

#endif // CONFIG_BT_NIMBLE_ENABLED
