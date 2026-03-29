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

// ---- Forward declarations ----

static void hidHostSubscribeNext(HidHostCtx& ctx);
static void hidHostStartRptRefRead(HidHostCtx& ctx);
static void hidHostReadReportMap(HidHostCtx& ctx);
static uint16_t getDescEndHandle(const HidHostCtx& ctx, uint16_t valHandle);

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

    // Only copy the keycodes that were actually present in the report.
    // Zero out any remaining slots so stale codes don't persist.
    std::memcpy(hid_host_prev_keys, curr, nkeys);
    if (nkeys < 6) std::memset(hid_host_prev_keys + nkeys, 0, 6 - nkeys);
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
        if (!hid_host_ctx->typeResolutionDone) {
            // Type resolution chain stalled (e.g. BLE_HS_EDONE never fired due to HCI error).
            // Force-complete so CCCD subscriptions always start regardless.
            LOGGER.warn("HID host: post-encryption delay — type resolution timed out, proceeding anyway");
            hid_host_ctx->typeResolutionDone = true;
            hid_host_ctx->subscribeIdx = 0;
        } else {
            LOGGER.info("HID host: post-encryption delay complete — starting CCCD subscriptions");
        }
        hidHostSubscribeNext(*hid_host_ctx);
    }
}

// ---- Report Map parsing ----
//
// Minimal HID Report Descriptor parser: extracts a mapping from Report ID → HidReportType.
// Handles short items only (long items are skipped). Tracks USAGE_PAGE, USAGE, REPORT_ID,
// and COLLECTION depth to classify each Application collection as Keyboard / Mouse / Consumer.
//
// Format of each short item: prefix byte = bTag(7:4) | bType(3:2) | bSize(1:0)
//   bType: 0=Main (COLLECTION, END_COLLECTION, INPUT, OUTPUT, FEATURE)
//          1=Global (USAGE_PAGE, REPORT_ID, ...)
//          2=Local  (USAGE, USAGE_MIN, USAGE_MAX, ...)
//   bSize: 0→0 bytes, 1→1 byte, 2→2 bytes, 3→4 bytes
//
// Long item prefix: 0xFE — skip entirely.
static void applyReportMapTypes(HidHostCtx& ctx) {
    const uint8_t* data = ctx.rptMap.data();
    size_t len = ctx.rptMap.size();

    uint16_t usagePage = 0;
    uint16_t usage     = 0;
    uint8_t  reportId  = 0;
    int      depth     = 0;
    HidReportType collType = HidReportType::Unknown;

    // typeMap: one entry per distinct non-zero reportId → type (used when REPORT_ID items present).
    // collOrder: one entry per Application Collection that has an INPUT, in descriptor order.
    //   Used when all inputRpts have reportId=0 (no REPORT_ID items in descriptor).
    struct Entry { uint8_t id; HidReportType type; };
    std::vector<Entry>         typeMap;
    std::vector<HidReportType> collOrder;
    bool collHadInput = false; // prevents recording the same collection twice

    size_t i = 0;
    while (i < len) {
        uint8_t prefix = data[i++];

        if (prefix == 0xFE) {          // Long item — skip
            if (i + 1 >= len) break;
            uint8_t lsz = data[i++];
            i++;                       // long tag byte
            i += lsz;
            continue;
        }

        uint8_t bSize   = prefix & 0x03;
        uint8_t bType   = (prefix >> 2) & 0x03;
        uint8_t bTag    = (prefix >> 4) & 0x0F;
        uint8_t dataLen = (bSize == 3) ? 4 : bSize;
        if (i + dataLen > len) break;

        uint32_t value = 0;
        for (uint8_t j = 0; j < dataLen; j++) value |= (uint32_t)data[i++] << (8 * j);

        if (bType == 0) {              // Main item
            if (bTag == 0xA) {         // COLLECTION
                if (depth == 0 && value == 0x01) { // Application collection at top level
                    // Classify by the USAGE_PAGE and USAGE set just before this COLLECTION.
                    // Common mappings per HID Usage Tables 1.4:
                    //   Generic Desktop (0x01) + Keyboard/Keypad (0x06) → Keyboard
                    //   Generic Desktop (0x01) + Mouse (0x02)            → Mouse
                    //   Consumer (0x0C)                                   → Consumer
                    if      (usagePage == 0x01 && usage == 0x06) collType = HidReportType::Keyboard;
                    else if (usagePage == 0x01 && usage == 0x02) collType = HidReportType::Mouse;
                    else if (usagePage == 0x0C)                  collType = HidReportType::Consumer;
                    else                                         collType = HidReportType::Unknown;
                    collHadInput = false;
                }
                depth++;
                usage = 0;             // local items reset after Main

            } else if (bTag == 0xC) { // END_COLLECTION
                if (depth > 0) depth--;
                if (depth == 0) { collType = HidReportType::Unknown; collHadInput = false; }
                usage = 0;

            } else if (bTag == 0x8) { // INPUT
                if (depth > 0 && collType != HidReportType::Unknown) {
                    // Ordered collection list: one entry per Application Collection with INPUT.
                    // Used for index-based matching when all reportIds are 0.
                    if (!collHadInput) {
                        collOrder.push_back(collType);
                        collHadInput = true;
                    }
                    // reportId map: one entry per distinct non-zero reportId.
                    // Used for ID-based matching when REPORT_ID items are present.
                    if (reportId != 0) {
                        bool found = false;
                        for (const auto& e : typeMap) { if (e.id == reportId) { found = true; break; } }
                        if (!found) typeMap.push_back({reportId, collType});
                    }
                }
                usage = 0;

            } else {
                usage = 0;             // OUTPUT / FEATURE — reset local items
            }

        } else if (bType == 1) {       // Global item
            if      (bTag == 0x0) usagePage = (uint16_t)value;
            else if (bTag == 0x8) reportId  = (uint8_t)value;

        } else if (bType == 2) {       // Local item
            if (bTag == 0x0) usage = (uint16_t)value;
        }
    }

    // Apply the resolved types to the discovered input reports.
    // Strategy: if any inputRpt has a non-zero reportId (REPORT_ID items present in descriptor),
    // match by reportId. Otherwise match by position in the Application Collection order —
    // this handles combo devices with multiple Input Report chars but no REPORT_ID prefix.
    bool anyNonZeroId = false;
    for (const auto& rpt : ctx.inputRpts) {
        if (rpt.reportId != 0) { anyNonZeroId = true; break; }
    }

    size_t zeroRptIdx = 0; // position counter for zero-reportId inputs (index into collOrder)
    for (auto& rpt : ctx.inputRpts) {
        if (anyNonZeroId) {
            // Match by reportId
            for (const auto& e : typeMap) {
                if (e.id == rpt.reportId) { rpt.type = e.type; break; }
            }
        } else {
            // All reportIds are 0 — match by position in Report Map collection order
            if (zeroRptIdx < collOrder.size()) rpt.type = collOrder[zeroRptIdx];
            zeroRptIdx++;
        }
        LOGGER.info("HID host: report val_handle={} reportId={} type={}",
                     rpt.valHandle, rpt.reportId, (int)rpt.type);
    }

    ctx.rptMap.clear(); // free the raw bytes — no longer needed
}

// ---- Report Reference (0x2908) read chain ----
//
// After all descriptor handles have been discovered, we read the value of each Report
// Reference descriptor to obtain the Report ID for each Input Report characteristic.
// The descriptor value is 2 bytes: [reportId, reportType] where reportType 1=Input.

static void hidHostStartRptRefRead(HidHostCtx& ctx) {
    // Advance past any reports that have no Report Reference descriptor
    while (ctx.rptRefReadIdx < (int)ctx.inputRpts.size() &&
           ctx.inputRpts[ctx.rptRefReadIdx].rptRefHandle == 0) {
        ctx.rptRefReadIdx++;
    }

    if (ctx.rptRefReadIdx >= (int)ctx.inputRpts.size()) {
        // All Report Reference reads complete — proceed to Report Map read
        hidHostReadReportMap(ctx);
        return;
    }

    uint16_t handle = ctx.inputRpts[ctx.rptRefReadIdx].rptRefHandle;
    int rc = ble_gattc_read(ctx.connHandle, handle, [](uint16_t conn_handle,
                             const struct ble_gatt_error* error,
                             struct ble_gatt_attr* attr, void* /*arg*/) -> int {
        if (!hid_host_ctx) return 0;
        auto& ctx = *hid_host_ctx;
        if (conn_handle != ctx.connHandle) return 0;

        if (error->status == BLE_HS_EDONE) {
            // Data was already processed in the status==0 callback; nothing to do.
            return 0;
        }

        if (error->status == 0 && attr != nullptr) {
            // Report Reference: [reportId (1 byte), reportType (1 byte)]
            // reportType: 1=Input, 2=Output, 3=Feature — we only care about Input here
            if (OS_MBUF_PKTLEN(attr->om) >= 2 &&
                ctx.rptRefReadIdx < (int)ctx.inputRpts.size()) {
                uint8_t rpt_ref[2] = {};
                os_mbuf_copydata(attr->om, 0, 2, rpt_ref);
                ctx.inputRpts[ctx.rptRefReadIdx].reportId = rpt_ref[0];
                LOGGER.info("HID host: report[{}] val_handle={} reportId={} reportType={}",
                             ctx.rptRefReadIdx,
                             ctx.inputRpts[ctx.rptRefReadIdx].valHandle,
                             rpt_ref[0], rpt_ref[1]);
            }
        }
        // Advance immediately on data (status==0) or ATT error — do NOT wait for BLE_HS_EDONE.
        // Under certain HCI error conditions (e.g. BLE_ERR_INV_HCI_CMD_PARMS from
        // LE_Add_Device_To_Resolving_List), NimBLE may never dispatch EDONE for ATT reads.
        ctx.rptRefReadIdx++;
        hidHostStartRptRefRead(ctx);
        return 0;
    }, nullptr);

    if (rc != 0) {
        LOGGER.warn("HID host: rptRef read[{}] failed rc={} — skipping", ctx.rptRefReadIdx, rc);
        ctx.rptRefReadIdx++;
        hidHostStartRptRefRead(ctx);
    }
}

// ---- Report Map (0x2A4B) read and type resolution ----
//
// Reads the Report Map characteristic using ATT Read Blob (read_long) to handle
// descriptors that exceed one ATT MTU. After the read completes, calls
// applyReportMapTypes() to populate HidReportType for each input report, then
// starts CCCD subscription.

static void hidHostReadReportMap(HidHostCtx& ctx) {
    if (ctx.rptMapHandle == 0) {
        LOGGER.info("HID host: no Report Map char — skipping type resolution");
        ctx.typeResolutionDone = true;
        ctx.subscribeIdx = 0;
        hidHostSubscribeNext(ctx);
        return;
    }

    int rc = ble_gattc_read_long(ctx.connHandle, ctx.rptMapHandle, 0,
        [](uint16_t conn_handle, const struct ble_gatt_error* error,
           struct ble_gatt_attr* attr, void* /*arg*/) -> int {
            if (!hid_host_ctx) return 0;
            auto& ctx = *hid_host_ctx;
            if (conn_handle != ctx.connHandle) return 0;

            if (error->status == 0 && attr != nullptr) {
                // Accumulate this blob into rptMap
                uint16_t chunk = OS_MBUF_PKTLEN(attr->om);
                size_t old_sz = ctx.rptMap.size();
                ctx.rptMap.resize(old_sz + chunk);
                os_mbuf_copydata(attr->om, 0, chunk, ctx.rptMap.data() + old_sz);
                return 0; // more blobs or BLE_HS_EDONE coming
            }

            // BLE_HS_EDONE or error
            if (!ctx.rptMap.empty()) {
                LOGGER.info("HID host: report map read ({} bytes)", ctx.rptMap.size());
                applyReportMapTypes(ctx);
            } else {
                LOGGER.warn("HID host: report map read failed or empty — types remain Unknown");
            }
            ctx.typeResolutionDone = true;
            ctx.subscribeIdx = 0;
            hidHostSubscribeNext(ctx);
            return 0;
        }, nullptr);

    if (rc != 0) {
        LOGGER.warn("HID host: report map read_long failed rc={} — skipping", rc);
        ctx.typeResolutionDone = true;
        ctx.subscribeIdx = 0;
        hidHostSubscribeNext(ctx);
    }
}

// ---- CCCD subscription chain ----

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
                auto lock = bt->getDataMutex().asScopedLock();
                lock.lock();
                for (const auto& r : bt->getScanResults()) {
                    if (r.addr == peer_addr) { name = r.name; break; }
                }
            }
            settings::PairedDevice device;
            device.addr        = peer_addr;
            device.profileId   = BT_PROFILE_HID_HOST;
            device.autoConnect = true; // default for new devices
            // Preserve user-configured fields (autoConnect) if record already exists
            const auto addr_hex = settings::addrToHex(peer_addr);
            settings::PairedDevice existing;
            if (settings::load(addr_hex, existing)) {
                device.autoConnect = existing.autoConnect;
            }
            // Always update the name (may have been resolved after first save)
            device.name = name;
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

// Descriptor discovery callback: find CCCD (0x2902) and Report Reference (0x2908)
// handles for each Input Report characteristic.
static int hidHostDscDiscCb(uint16_t conn_handle, const struct ble_gatt_error* error,
                             uint16_t chr_val_handle, const struct ble_gatt_dsc* dsc, void* /*arg*/) {
    if (!hid_host_ctx) return 0;
    auto& ctx = *hid_host_ctx;
    if (conn_handle != ctx.connHandle) return 0;

    if (error->status == 0 && dsc != nullptr) {
        uint16_t dsc_uuid = ble_uuid_u16(&dsc->uuid.u);
        for (auto& rpt : ctx.inputRpts) {
            if (rpt.valHandle != chr_val_handle) continue;
            if (dsc_uuid == 0x2902) {
                rpt.cccdHandle = dsc->handle;
                LOGGER.info("HID host: CCCD handle={} for val_handle={}", dsc->handle, chr_val_handle);
            } else if (dsc_uuid == 0x2908) {
                rpt.rptRefHandle = dsc->handle;
                LOGGER.info("HID host: rptRef handle={} for val_handle={}", dsc->handle, chr_val_handle);
            }
            break;
        }
    } else if (error->status == BLE_HS_EDONE) {
        // Move to next input report's descriptor discovery
        int next_idx = ctx.dscDiscIdx + 1;

        if (next_idx < (int)ctx.inputRpts.size()) {
            ctx.dscDiscIdx = next_idx;
            auto& next_rpt = ctx.inputRpts[next_idx];
            // Bound to the next characteristic declaration (not just the next Input Report).
            uint16_t end = getDescEndHandle(ctx, next_rpt.valHandle);
            int rc = ble_gattc_disc_all_dscs(ctx.connHandle, next_rpt.valHandle, end,
                                              hidHostDscDiscCb, nullptr);
            if (rc != 0) {
                LOGGER.warn("HID host: disc_all_dscs[{}] failed rc={}", next_idx, rc);
                ctx.rptRefReadIdx = 0;
                hidHostStartRptRefRead(ctx);
            }
        } else {
            // All descriptor discovery done — read Report Reference values, then Report Map
            ctx.rptRefReadIdx = 0;
            hidHostStartRptRefRead(ctx);
        }
    }
    return 0;
}

// Returns the upper bound (inclusive) for descriptor discovery for the characteristic
// at val_handle. Descriptors occupy [val_handle+1, next_chr_def_handle-1]. Uses the
// sorted allChrDefHandles list (populated during chr discovery) to find the next
// characteristic boundary. Falls back to hidSvcEnd if no later chr is known.
static uint16_t getDescEndHandle(const HidHostCtx& ctx, uint16_t valHandle) {
    for (uint16_t dh : ctx.allChrDefHandles) {
        if (dh > valHandle) return dh - 1;
    }
    return ctx.hidSvcEnd;
}

// Characteristic discovery callback: collect Input Report chars (UUID 0x2A4D, NOTIFY)
static int hidHostChrDiscCb(uint16_t conn_handle, const struct ble_gatt_error* error,
                             const struct ble_gatt_chr* chr, void* arg) {
    if (!hid_host_ctx) return 0;
    auto& ctx = *hid_host_ctx;
    if (conn_handle != ctx.connHandle) return 0;

    if (error->status == 0 && chr != nullptr) {
        // Track ALL characteristic definition handles so we can accurately bound descriptor
        // discovery ranges later. Without this, a char that is last (or only) in the service
        // would use hidSvcEnd=65535 as end handle, sweeping all subsequent characteristics.
        ctx.allChrDefHandles.push_back(chr->def_handle);

        uint16_t uuid16 = ble_uuid_u16(&chr->uuid.u);
        if (uuid16 == 0x2A4D && (chr->properties & BLE_GATT_CHR_PROP_NOTIFY)) {
            // Input Report — collect for subscription and type resolution
            HidHostInputRpt rpt = {};
            rpt.valHandle    = chr->val_handle;
            rpt.cccdHandle   = 0;
            rpt.rptRefHandle = 0;
            rpt.reportId     = 0;
            rpt.type         = HidReportType::Unknown;
            ctx.inputRpts.push_back(rpt);
            LOGGER.info("HID host: Input Report chr val_handle={}", chr->val_handle);
        } else if (uuid16 == 0x2A4B) {
            // Report Map — save handle so we can read it after descriptor discovery
            ctx.rptMapHandle = chr->val_handle;
            LOGGER.info("HID host: Report Map chr val_handle={}", chr->val_handle);
        }
    } else if (error->status == BLE_HS_EDONE) {
        // Sort def handles ascending so getDescEndHandle() binary-searches correctly.
        std::sort(ctx.allChrDefHandles.begin(), ctx.allChrDefHandles.end());

        if (ctx.inputRpts.empty()) {
            LOGGER.warn("HID host: no Input Report chars — disconnecting");
            ble_gap_terminate(ctx.connHandle, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }
        // Discover descriptors (CCCD + Report Reference) for the first input report.
        // End handle is bounded to the declaration of the next characteristic so we
        // don't accidentally pick up descriptors that belong to other chars.
        ctx.dscDiscIdx = 0;
        auto& first = ctx.inputRpts[0];
        uint16_t end = getDescEndHandle(ctx, first.valHandle);
        int rc = ble_gattc_disc_all_dscs(ctx.connHandle, first.valHandle, end,
                                          hidHostDscDiscCb, nullptr);
        if (rc != 0) {
            LOGGER.warn("HID host: disc_all_dscs[0] failed rc={}", rc);
            ctx.rptRefReadIdx = 0;
            hidHostStartRptRefRead(ctx);
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
                QueueHandle_t saved_queue = hid_host_key_queue;
                hid_host_ctx.reset();
                hid_host_key_queue = nullptr; // null immediately; actual delete deferred below
                std::memset(hid_host_prev_keys, 0, sizeof(hid_host_prev_keys));
                hid_host_mouse_x.store(0);
                hid_host_mouse_y.store(0);
                hid_host_mouse_btn.store(false);
                hid_host_mouse_active.store(false);
                // Defer both LVGL indev deletion AND key queue deletion to the main task.
                // hidHostKeyboardReadCb runs on the LVGL task and calls xQueueReceive —
                // deleting the queue here on the NimBLE task would race with that callback.
                // Deleting the indev first (inside the LVGL lock) ensures the read callback
                // is never called again before the queue is deleted.
                getMainDispatcher().dispatch([saved_kb, saved_mouse, saved_cursor, saved_queue] {
                    if (!tt::lvgl::lock(1000)) {
                        LOGGER.warn("HID host: failed to acquire LVGL lock for indev cleanup");
                        if (saved_queue) vQueueDelete(saved_queue);
                        return;
                    }
                    if (saved_kb) {
                        tt::lvgl::hardware_keyboard_set_indev(nullptr);
                        lv_indev_delete(saved_kb);
                    }
                    if (saved_mouse)  lv_indev_delete(saved_mouse);
                    if (saved_cursor) lv_obj_delete(saved_cursor);
                    tt::lvgl::unlock();
                    // Delete the queue after the indev is gone — no more callbacks can fire.
                    if (saved_queue) vQueueDelete(saved_queue);
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
                // Max HID report size: 64 bytes is more than sufficient for
                // keyboard (8), mouse (4-8), consumer (2-4), or combo reports (≤16).
                if (len > 0 && len <= 64) {
                    uint8_t buf[64] = {};
                    os_mbuf_copydata(event->notify_rx.om, 0, len, buf);
                    for (const auto& rpt : ctx.inputRpts) {
                        if (rpt.valHandle != event->notify_rx.attr_handle) continue;

                        // In BLE HID (HOGP), the Report ID is carried by the Report Reference
                        // descriptor (0x2908) only — it is NOT prepended to notification data.
                        // HOGP §4.4: "The Report ID is NOT included in the value of the
                        // characteristic." Pass the full notification payload as-is.
                        const uint8_t* payload     = buf;
                        uint16_t       payload_len = len;

                        switch (rpt.type) {
                            case HidReportType::Keyboard:
                                hidHostHandleKeyboardReport(payload, payload_len);
                                break;
                            case HidReportType::Mouse:
                                hidHostHandleMouseReport(payload, payload_len);
                                break;
                            case HidReportType::Consumer:
                                // Consumer/media keys — not yet handled; log for diagnostics
                                LOGGER.info("HID host: consumer report len={}", payload_len);
                                break;
                            case HidReportType::Unknown:
                                // Report Map not available or parsing failed.
                                // Fall back to length heuristic as a best-effort.
                                if      (payload_len >= 6) hidHostHandleKeyboardReport(payload, payload_len);
                                else if (payload_len >= 3) hidHostHandleMouseReport(payload, payload_len);
                                break;
                        }
                        break;
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
        auto lock = bt->getDataMutex().asScopedLock();
        for (const auto& sa : bt->getScanAddresses()) {
            if (std::memcmp(sa.val, addr.data(), 6) == 0) {
                ble_addr.type = sa.type;
                break;
            }
        }
    }

    uint8_t own_addr_type;
    if (ble_hs_id_infer_auto(0, &own_addr_type) != 0) {
        LOGGER.warn("hidHostConnect: failed to infer own address type, using PUBLIC");
        own_addr_type = BLE_OWN_ADDR_PUBLIC;
    }

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
