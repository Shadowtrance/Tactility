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

#include <Tactility/service/bluetooth/BluetoothNimBLEInternal.h>

// ble_store_config_init() is not declared in the public header in this IDF version
// but exists in the store/config library — forward-declare it here.
extern "C" void ble_store_config_init(void);

namespace tt::service::bluetooth {

static const auto LOGGER = Logger("BtService");

// ---- Bluetooth singleton ----

std::shared_ptr<Bluetooth> bt_singleton;

// ---- Forward declarations (internal to this file) ----

static void bleHostTask(void* param);
static void onSync();
static void onReset(int reason);
static void dispatchDisable(std::shared_ptr<Bluetooth> bt);
static int gapEventHandler(struct ble_gap_event* event, void* arg);

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
void scheduleAdvRestart(std::shared_ptr<Bluetooth> bt, uint64_t delay_us) {
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

    // Sync GATT handle values back to bt members and module-level vars
    sppInitGattHandles();
    midiInitGattHandles();
    hidDeviceInitGattHandles();

    bt->setRadioState(RadioState::On);
    publishEvent(bt, BtEvent::RadioStateOn);

    // Auto-start profile servers that were active before the last reboot.
    // This lets bonded centrals (phone, PC) reconnect without user intervention.
    if (settings::shouldMidiAutoStart()) {
        LOGGER.info("Auto-starting MIDI server");
        midiStartInternal();
        return; // midiStartInternal calls startAdvertising — no need for name-only advert below
    }
    if (settings::shouldSppAutoStart()) {
        LOGGER.info("Auto-starting SPP server");
        sppStartInternal();
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
void startAdvertising(const ble_uuid128_t* svcUuid) {
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
void startAdvertisingHid(uint16_t appearance) {
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
    hidDeviceInitGatt();

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

void publishEvent(std::shared_ptr<Bluetooth> bt, BtEvent event) {
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
