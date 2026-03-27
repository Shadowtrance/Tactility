#ifdef ESP_PLATFORM
#include <sdkconfig.h>
#endif

#if defined(CONFIG_BT_NIMBLE_ENABLED)

#include <Tactility/service/bluetooth/BluetoothNimBLEInternal.h>

namespace tt::service::bluetooth {

static const auto LOGGER = Logger("BtService");

// ---- HID Host globals ----

std::unique_ptr<HidHostCtx> hid_host_ctx;
static QueueHandle_t hid_host_key_queue = nullptr;
static uint8_t hid_host_prev_keys[6] = {}; // previous keyboard report keycodes for press/release diff
// One-shot timer to delay CCCD writes after ENC_CHANGE (bonded devices need time to settle)
esp_timer_handle_t hid_enc_retry_timer = nullptr;

// Mouse state — written from NimBLE task, read from LVGL task (atomic)
static std::atomic<int32_t> hid_host_mouse_x{0};
static std::atomic<int32_t> hid_host_mouse_y{0};
static std::atomic<bool>    hid_host_mouse_btn{false};
static std::atomic<bool>    hid_host_mouse_active{false}; // set on first movement; shows cursor

#define HID_HOST_KEY_QUEUE_SIZE 64
struct HidHostKeyEvt { uint32_t key; bool pressed; };

// ---- Forward declaration ----

static void hidHostSubscribeNext(HidHostCtx& ctx);

// ---- HID Host implementation ----

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

// Timer callback: fires 500ms after ENC_CHANGE to let the device settle before we write CCCDs.
// Called from the esp_timer task — safe for NimBLE GATT calls (same as advRestartCallback).
void hidEncRetryTimerCb(void* /*arg*/) {
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
int hidHostGapCb(struct ble_gap_event* event, void* arg) {
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

bool hidHostIsConnectedImpl() {
    return hid_host_ctx != nullptr &&
           hid_host_ctx->connHandle != BLE_HS_CONN_HANDLE_NONE &&
           !hid_host_ctx->inputRpts.empty() &&
           hid_host_ctx->subscribeIdx >= (int)hid_host_ctx->inputRpts.size();
}

// hidEncRetryTimerCb is defined above; the timer handle (hid_enc_retry_timer) is
// created/destroyed in BluetoothService::onStart/onStop in BluetoothNimBLE.cpp.
// Expose a creation helper so the service can pass the right callback.
// (The service creates the timer itself; this function is not needed separately.)

} // namespace tt::service::bluetooth

#endif // CONFIG_BT_NIMBLE_ENABLED
