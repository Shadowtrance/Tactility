#include "usb_host_hid_module.h"

#include <tactility/module.h>
#include <tactility/log.h>

#include <atomic>
#include <cstring>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <usb/hid_host.h>
#include <usb/hid_usage_keyboard.h>
#include <usb/hid_usage_mouse.h>
#include <esp_timer.h>

#include <lvgl.h>
#include <esp_lvgl_port.h>
#include <Tactility/Assets.h>
#include <Tactility/lvgl/Keyboard.h>

#define TAG "UsbHid"

// Key event passed from HID callback to LVGL read callback
typedef struct {
    uint32_t lv_key;
    bool pressed;
} key_event_t;

constexpr auto KEY_QUEUE_SIZE = 64;
constexpr uint32_t KEY_REPEAT_DELAY_MS = 500;   // delay before first repeat
constexpr uint32_t KEY_REPEAT_RATE_MS  = 50;    // interval between repeats (~20/sec)
constexpr auto HID_EVENT_QUEUE_SIZE = 8;
constexpr auto HID_PROC_TASK_STACK = 4096;
constexpr auto HID_PROC_TASK_PRIORITY = 5;
constexpr auto HID_STOP_TIMEOUT_MS = 2000;

// ---- Shared state (written from HID bg task, read from LVGL task) ----
static std::atomic<int32_t> mouse_x{0};
static std::atomic<int32_t> mouse_y{0};
static std::atomic<bool> mouse_pressed{false};
static std::atomic<bool> mouse_connected{false};
static QueueHandle_t key_queue = nullptr;

// ---- HID device event queue (connect events must be processed outside HID bg task) ----
typedef struct {
    hid_host_device_handle_t handle;
    hid_host_driver_event_t event;
    void* arg;
} hid_dev_event_t;

static QueueHandle_t hid_event_queue = nullptr;
static TaskHandle_t hid_proc_task_handle = nullptr;
static SemaphoreHandle_t hid_proc_task_done = nullptr;
static std::atomic<bool> hid_proc_running{false};

// ---- LVGL input devices ----
static lv_indev_t* mouse_indev = nullptr;
static lv_indev_t* kb_indev = nullptr;
static lv_obj_t* mouse_cursor = nullptr;

// ---- Keyboard state (file-scope so disconnect can reset them) ----
static uint8_t prev_keys[HID_KEYBOARD_KEY_MAX] = {};
// Maps HID keycode → the lv_key that was sent on press; used to send the matching
// release with the same key code regardless of which slot the key occupies.
static uint32_t pressed_lv_keys[256] = {};
static bool caps_lock_active = false;
static bool num_lock_active = true;   // most keyboards power on with NumLock on
static bool scroll_lock_active = false;

// ---- Keyboard device handle (written from hidProcTask, read from hidProcTask for LED updates) ----
static hid_host_device_handle_t kb_handle = nullptr;
static std::atomic<bool> kb_led_pending{false};

// ---- Software key repeat (written from HID task, read from LVGL task) ----
// repeat_lv_key==0 means no key is currently held for repeat.
static std::atomic<uint32_t> repeat_lv_key{0};
static std::atomic<uint32_t> repeat_start_ms{0};
static std::atomic<uint32_t> repeat_last_ms{0};

// ---- HID keycode to ASCII table (from ESP-IDF HID example) ----
static const uint8_t keycode2ascii[57][2] = {
    {0, 0},       // HID_KEY_NO_PRESS
    {0, 0},       // HID_KEY_ROLLOVER
    {0, 0},       // HID_KEY_POST_FAIL
    {0, 0},       // HID_KEY_ERROR_UNDEFINED
    {'a', 'A'}, {'b', 'B'}, {'c', 'C'}, {'d', 'D'}, {'e', 'E'},
    {'f', 'F'}, {'g', 'G'}, {'h', 'H'}, {'i', 'I'}, {'j', 'J'},
    {'k', 'K'}, {'l', 'L'}, {'m', 'M'}, {'n', 'N'}, {'o', 'O'},
    {'p', 'P'}, {'q', 'Q'}, {'r', 'R'}, {'s', 'S'}, {'t', 'T'},
    {'u', 'U'}, {'v', 'V'}, {'w', 'W'}, {'x', 'X'}, {'y', 'Y'},
    {'z', 'Z'},
    {'1', '!'}, {'2', '@'}, {'3', '#'}, {'4', '$'}, {'5', '%'},
    {'6', '^'}, {'7', '&'}, {'8', '*'}, {'9', '('}, {'0', ')'},
    {'\r', '\r'}, // HID_KEY_ENTER (0x28)
    {0, 0},       // HID_KEY_ESC   (0x29) — handled in switch
    {'\b', 0},    // HID_KEY_DEL   (0x2A) — physical Backspace; handled in switch
    {'\t', '\t'}, // HID_KEY_TAB   (0x2B) — handled in switch
    {' ', ' '},   // HID_KEY_SPACE (0x2C)
    {'-', '_'}, {'=', '+'}, {'[', '{'}, {']', '}'},
    {'\\', '|'}, {'\\', '|'}, {';', ':'}, {'\'', '"'},
    {'`', '~'}, {',', '<'}, {'.', '>'}, {'/', '?'},
};

static uint32_t hid_keycode_to_lv_key(uint8_t modifier, uint8_t key_code,
                                       bool caps_lock, bool num_lock) {
    bool shift = (modifier & HID_LEFT_SHIFT) || (modifier & HID_RIGHT_SHIFT);
    bool ctrl  = (modifier & HID_LEFT_CONTROL) || (modifier & HID_RIGHT_CONTROL);
    bool alt   = (modifier & HID_LEFT_ALT) || (modifier & HID_RIGHT_ALT);

    // Navigation / action keys — always active even with modifiers held
    switch (key_code) {
        case HID_KEY_ENTER:          return LV_KEY_ENTER;
        case HID_KEY_ESC:            return LV_KEY_ESC;
        case HID_KEY_DEL:            return LV_KEY_BACKSPACE;   // physical Backspace key
        case HID_KEY_DELETE:         return LV_KEY_DEL;         // physical Delete key
        case HID_KEY_TAB:            return shift ? LV_KEY_PREV : LV_KEY_NEXT;
        case HID_KEY_UP:             return LV_KEY_UP;
        case HID_KEY_DOWN:           return LV_KEY_DOWN;
        case HID_KEY_LEFT:           return LV_KEY_LEFT;
        case HID_KEY_RIGHT:          return LV_KEY_RIGHT;
        case HID_KEY_HOME:           return LV_KEY_HOME;
        case HID_KEY_END:            return LV_KEY_END;

        // Numpad — Enter and arithmetic operators are NumLock-independent
        case HID_KEY_KEYPAD_ENTER:   return LV_KEY_ENTER;
        case HID_KEY_KEYPAD_ADD:     return '+';
        case HID_KEY_KEYPAD_SUB:     return '-';
        case HID_KEY_KEYPAD_MUL:     return '*';
        case HID_KEY_KEYPAD_DIV:     return '/';

        // Numpad digits: digits when NumLock on, navigation when off
        // Cast LV_KEY_* enum to uint32_t to match the char literal branch type.
        case HID_KEY_KEYPAD_0:       return num_lock ? (uint32_t)'0' : 0u;                       // off: Insert (no LVGL key)
        case HID_KEY_KEYPAD_1:       return num_lock ? (uint32_t)'1' : (uint32_t)LV_KEY_END;
        case HID_KEY_KEYPAD_2:       return num_lock ? (uint32_t)'2' : (uint32_t)LV_KEY_DOWN;
        case HID_KEY_KEYPAD_3:       return num_lock ? (uint32_t)'3' : 0u;                       // off: PgDn (handled as scroll)
        case HID_KEY_KEYPAD_4:       return num_lock ? (uint32_t)'4' : (uint32_t)LV_KEY_LEFT;
        case HID_KEY_KEYPAD_5:       return num_lock ? (uint32_t)'5' : 0u;
        case HID_KEY_KEYPAD_6:       return num_lock ? (uint32_t)'6' : (uint32_t)LV_KEY_RIGHT;
        case HID_KEY_KEYPAD_7:       return num_lock ? (uint32_t)'7' : (uint32_t)LV_KEY_HOME;
        case HID_KEY_KEYPAD_8:       return num_lock ? (uint32_t)'8' : (uint32_t)LV_KEY_UP;
        case HID_KEY_KEYPAD_9:       return num_lock ? (uint32_t)'9' : 0u;                       // off: PgUp (handled as scroll)
        case HID_KEY_KEYPAD_DELETE:  return num_lock ? (uint32_t)'.' : (uint32_t)LV_KEY_DEL;    // numpad . / Del

        default: break;
    }

    // Suppress printable output when Ctrl or Alt is held (avoid typing 'c' on Ctrl+C etc.)
    if (ctrl || alt) return 0;

    // Printable characters — indexed directly by HID keycode (table covers 0x00–0x38)
    if (key_code < (sizeof(keycode2ascii) / sizeof(keycode2ascii[0]))) {
        // Caps Lock flips shift for letter keys only; non-letter keys are shift-only
        bool is_letter = (key_code >= 0x04 && key_code <= 0x1D);
        bool effective_shift = is_letter ? (shift ^ caps_lock) : shift;
        uint8_t ch = keycode2ascii[key_code][effective_shift ? 1 : 0];
        if (ch != 0) return (uint32_t)ch;
    }
    return 0;
}

// Enqueue N press+release pairs of scroll_key into key_queue
static void enqueue_scroll(uint32_t scroll_key, int ticks) {
    if (!key_queue) return;
    for (int t = 0; t < ticks; t++) {
        key_event_t press   = { scroll_key, true  };
        key_event_t release = { scroll_key, false };
        xQueueSend(key_queue, &press,   0);
        xQueueSend(key_queue, &release, 0);
    }
}

// ---- Interface callback: called from HID bg task for input reports ----
static void hid_interface_callback(hid_host_device_handle_t handle,
                                   const hid_host_interface_event_t event,
                                   void* /*arg*/)
{
    uint8_t data[64] = {};
    size_t data_len = 0;
    hid_host_dev_params_t params;

    if (hid_host_device_get_params(handle, &params) != ESP_OK) {
        return;
    }

    switch (event) {
    case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
        if (hid_host_device_get_raw_input_report_data(handle, data, sizeof(data), &data_len) != ESP_OK) {
            break;
        }
        LOG_D(TAG, "INPUT_REPORT proto=%d len=%d data[0..3]=%02x %02x %02x %02x",
              params.proto, (int)data_len,
              data_len > 0 ? data[0] : 0, data_len > 1 ? data[1] : 0,
              data_len > 2 ? data[2] : 0, data_len > 3 ? data[3] : 0);

        if (params.proto == HID_PROTOCOL_KEYBOARD) {
            if (data_len < sizeof(hid_keyboard_input_report_boot_t)) break;
            auto* kb = reinterpret_cast<const hid_keyboard_input_report_boot_t*>(data);

            for (int i = 0; i < HID_KEYBOARD_KEY_MAX; i++) {
                // ---- Release check: was a key in slot i of the previous report released? ----
                uint8_t prev_hid = prev_keys[i];
                if (prev_hid > HID_KEY_ERROR_UNDEFINED) {
                    bool still_pressed = false;
                    for (int j = 0; j < HID_KEYBOARD_KEY_MAX; j++) {
                        if (kb->key[j] == prev_hid) { still_pressed = true; break; }
                    }
                    if (!still_pressed) {
                        // Look up the lv_key by HID code — slot-independent, handles
                        // keys that moved to a different slot between reports.
                        uint32_t lv_key = pressed_lv_keys[prev_hid];
                        pressed_lv_keys[prev_hid] = 0;
                        if (lv_key && key_queue) {
                            key_event_t evt = { lv_key, false };
                            xQueueSend(key_queue, &evt, 0);
                        }
                        // Stop repeating if this was the held key
                        if (lv_key && lv_key == repeat_lv_key.load()) {
                            repeat_lv_key.store(0);
                        }
                    }
                }

                // ---- Press check: is there a newly pressed key in slot i? ----
                uint8_t hid_code = kb->key[i];
                if (hid_code > HID_KEY_ERROR_UNDEFINED) {
                    bool was_pressed = false;
                    for (int j = 0; j < HID_KEYBOARD_KEY_MAX; j++) {
                        if (prev_keys[j] == hid_code) { was_pressed = true; break; }
                    }
                    if (!was_pressed) {
                        // Toggle lock keys (don't send to LVGL — schedule LED update instead)
                        if (hid_code == HID_KEY_CAPS_LOCK) {
                            caps_lock_active = !caps_lock_active;
                            LOG_D(TAG, "Caps Lock: %s", caps_lock_active ? "ON" : "OFF");
                            kb_led_pending.store(true);
                            continue;
                        }
                        if (hid_code == HID_KEY_NUM_LOCK) {
                            num_lock_active = !num_lock_active;
                            LOG_D(TAG, "Num Lock: %s", num_lock_active ? "ON" : "OFF");
                            kb_led_pending.store(true);
                            continue;
                        }
                        if (hid_code == HID_KEY_SCROLL_LOCK) {
                            scroll_lock_active = !scroll_lock_active;
                            LOG_D(TAG, "Scroll Lock: %s", scroll_lock_active ? "ON" : "OFF");
                            kb_led_pending.store(true);
                            continue;
                        }
                        // Page Up / Page Down → scroll the focused widget (8 key-pair bursts)
                        bool is_pgup = (hid_code == HID_KEY_PAGEUP) ||
                                       (!num_lock_active && hid_code == HID_KEY_KEYPAD_9);
                        bool is_pgdn = (hid_code == HID_KEY_PAGEDOWN) ||
                                       (!num_lock_active && hid_code == HID_KEY_KEYPAD_3);
                        if (is_pgup || is_pgdn) {
                            enqueue_scroll(is_pgup ? LV_KEY_UP : LV_KEY_DOWN, 8);
                            // Not stored in pressed_lv_keys — no held-release event needed
                            continue;
                        }
                        // Normal key
                        uint32_t lv_key = hid_keycode_to_lv_key(kb->modifier.val, hid_code,
                                                                  caps_lock_active, num_lock_active);
                        LOG_D(TAG, "key press: hid=0x%02x mod=0x%02x lv_key=0x%" PRIx32,
                              hid_code, kb->modifier.val, lv_key);
                        if (lv_key && key_queue) {
                            key_event_t evt = { lv_key, true };
                            xQueueSend(key_queue, &evt, 0);
                            pressed_lv_keys[hid_code] = lv_key;
                            // Arm repeat for this key
                            repeat_lv_key.store(lv_key);
                            repeat_start_ms.store((uint32_t)(esp_timer_get_time() / 1000));
                            repeat_last_ms.store(0);
                        }
                    }
                }
            }
            memcpy(prev_keys, kb->key, HID_KEYBOARD_KEY_MAX);

        } else if (params.proto == HID_PROTOCOL_MOUSE) {
            if (data_len < sizeof(hid_mouse_input_report_boot_t)) break;
            auto* ms = reinterpret_cast<const hid_mouse_input_report_boot_t*>(data);
            lv_display_t* disp = lv_display_get_default();
            if (disp) {
                constexpr int32_t CURSOR_SIZE = 16;
                int32_t w = lv_display_get_horizontal_resolution(disp);
                int32_t h = lv_display_get_vertical_resolution(disp);
                int32_t nx = mouse_x + ms->x_displacement;
                int32_t ny = mouse_y + ms->y_displacement;
                if (nx < 0) nx = 0;
                if (nx > w - CURSOR_SIZE - 1) nx = w - CURSOR_SIZE - 1;
                if (ny < 0) ny = 0;
                if (ny > h - CURSOR_SIZE - 1) ny = h - CURSOR_SIZE - 1;
                mouse_x = nx;
                mouse_y = ny;
            }
            mouse_pressed = ms->buttons.button1;

            // Right-click → ESC
            static bool prev_button2 = false;
            if (ms->buttons.button2 != prev_button2) {
                key_event_t evt = { LV_KEY_ESC, ms->buttons.button2 != 0 };
                if (key_queue) xQueueSend(key_queue, &evt, 0);
                prev_button2 = ms->buttons.button2 != 0;
            }

            // Scroll wheel: many mice send a 4th byte even in boot protocol
            if (data_len > sizeof(hid_mouse_input_report_boot_t) && key_queue) {
                int8_t wheel = (int8_t)data[sizeof(hid_mouse_input_report_boot_t)];
                if (wheel != 0) {
                    uint32_t scroll_key = (wheel < 0) ? LV_KEY_UP : LV_KEY_DOWN;
                    int ticks = (wheel < 0) ? -wheel : wheel;
                    if (ticks > 8) ticks = 8;
                    enqueue_scroll(scroll_key, ticks);
                }
            }
        }
        break;

    case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
        LOG_I(TAG, "HID device disconnected (proto=%d)", params.proto);
        if (params.proto == HID_PROTOCOL_KEYBOARD) {
            // Clear key state so stale keys don't generate spurious releases after reconnect
            memset(prev_keys, 0, sizeof(prev_keys));
            memset(pressed_lv_keys, 0, sizeof(pressed_lv_keys));
            repeat_lv_key.store(0);
            kb_handle = nullptr;
            kb_led_pending.store(false);
            // caps_lock_active / num_lock_active intentionally preserved across reconnects
            if (lvgl_port_lock(0)) {
                tt::lvgl::hardware_keyboard_set_indev(nullptr);
                lvgl_port_unlock();
            }
        } else if (params.proto == HID_PROTOCOL_MOUSE) {
            mouse_connected = false;
            if (mouse_cursor && lvgl_port_lock(0)) {
                lv_obj_add_flag(mouse_cursor, LV_OBJ_FLAG_HIDDEN);
                lvgl_port_unlock();
            }
        }
        hid_host_device_close(handle);
        break;

    case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
        LOG_W(TAG, "HID transfer error (proto=%d)", params.proto);
        break;

    default:
        break;
    }
}

// ---- LVGL read callbacks ----
static void mouse_read_cb(lv_indev_t* /*indev*/, lv_indev_data_t* data) {
    int32_t cx = mouse_x;
    int32_t cy = mouse_y;

    // lv_indev_proc_pointer (lv_indev.c) applies the display rotation transform to all
    // pointer indev coordinates. It uses disp->hor_res / disp->ver_res directly (the raw,
    // unswapped native screen dimensions). Our cursor is accumulated in logical space, so
    // pre-apply the inverse transform so LVGL's forward transform cancels it out.
    // Use lv_display_get_original_*_resolution which returns the raw disp->hor_res/ver_res
    // without any rotation swap — matching exactly what lv_indev.c uses.
    lv_display_t* disp = lv_display_get_default();
    if (disp) {
        int32_t ow = lv_display_get_original_horizontal_resolution(disp);  // raw disp->hor_res
        int32_t oh = lv_display_get_original_vertical_resolution(disp);    // raw disp->ver_res
        switch (lv_display_get_rotation(disp)) {
            case LV_DISPLAY_ROTATION_0:
                data->point.x = (lv_coord_t)cx;
                data->point.y = (lv_coord_t)cy;
                break;
            case LV_DISPLAY_ROTATION_90:
                // LVGL: (oh - y_in - 1, x_in). Inverse: x_in=cy, y_in=oh-cx-1
                data->point.x = (lv_coord_t)cy;
                data->point.y = (lv_coord_t)(oh - cx - 1);
                break;
            case LV_DISPLAY_ROTATION_180:
                // LVGL: (ow - x_in - 1, oh - y_in - 1). Inverse: x_in=ow-cx-1, y_in=oh-cy-1
                data->point.x = (lv_coord_t)(ow - cx - 1);
                data->point.y = (lv_coord_t)(oh - cy - 1);
                break;
            case LV_DISPLAY_ROTATION_270:
                // LVGL: (y_in, ow - x_in - 1). Inverse: x_in=ow-cy-1, y_in=cx
                data->point.x = (lv_coord_t)(ow - cy - 1);
                data->point.y = (lv_coord_t)cx;
                break;
        }
    } else {
        data->point.x = (lv_coord_t)cx;
        data->point.y = (lv_coord_t)cy;
    }

    data->state = mouse_pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

static void keyboard_read_cb(lv_indev_t* /*indev*/, lv_indev_data_t* data) {
    // Emit the release half of a synthetic repeat cycle (set by repeat logic below)
    static bool emit_repeat_release = false;
    static uint32_t repeat_release_key = 0;
    if (emit_repeat_release) {
        emit_repeat_release = false;
        data->key = repeat_release_key;
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    // Drain real events from the HID task first
    key_event_t evt;
    if (key_queue && xQueueReceive(key_queue, &evt, 0) == pdTRUE) {
        data->key = evt.lv_key;
        data->state = evt.pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
        data->continue_reading = (uxQueueMessagesWaiting(key_queue) > 0);
        return;
    }

    // Software key repeat: generate press+release pairs while a key is held
    uint32_t rkey = repeat_lv_key.load();
    if (rkey != 0) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        uint32_t elapsed = now_ms - repeat_start_ms.load();
        if (elapsed >= KEY_REPEAT_DELAY_MS) {
            uint32_t last = repeat_last_ms.load();
            if (last == 0 || (now_ms - last) >= KEY_REPEAT_RATE_MS) {
                repeat_last_ms.store(now_ms);
                // Emit press now; schedule the paired release for the very next call
                emit_repeat_release = true;
                repeat_release_key  = rkey;
                data->key   = rkey;
                data->state = LV_INDEV_STATE_PRESSED;
                data->continue_reading = true;  // ensure LVGL calls us again for the release
                return;
            }
        }
    }

    data->state = LV_INDEV_STATE_RELEASED;
}

// ---- Device event processing task ----
static void hidProcTask(void* /*arg*/) {
    LOG_I(TAG, "HID proc task started");

    // Modules start before LVGL is initialized; wait for it to be ready
    vTaskDelay(pdMS_TO_TICKS(3000));

    if (lvgl_port_lock(0)) {
        mouse_cursor = lv_image_create(lv_layer_sys());
        lv_obj_remove_flag(mouse_cursor, LV_OBJ_FLAG_CLICKABLE);
        lv_image_set_src(mouse_cursor, TT_ASSETS_UI_CURSOR);
        lv_obj_add_flag(mouse_cursor, LV_OBJ_FLAG_HIDDEN);  // hidden until mouse connects

        mouse_indev = lv_indev_create();
        lv_indev_set_type(mouse_indev, LV_INDEV_TYPE_POINTER);
        lv_indev_set_read_cb(mouse_indev, mouse_read_cb);
        lv_indev_set_cursor(mouse_indev, mouse_cursor);

        kb_indev = lv_indev_create();
        lv_indev_set_type(kb_indev, LV_INDEV_TYPE_KEYPAD);
        lv_indev_set_read_cb(kb_indev, keyboard_read_cb);
        // Do NOT call hardware_keyboard_set_indev here — only register it when a
        // physical keyboard actually connects, so the software keyboard is shown
        // when no USB keyboard is attached. But DO assign to the current default
        // group so mouse right-click/scroll key events (via key_queue) are delivered.
        lv_indev_set_group(kb_indev, lv_group_get_default());
        lvgl_port_unlock();
        LOG_I(TAG, "LVGL input devices registered");
    } else {
        LOG_W(TAG, "Could not acquire LVGL lock for indev registration");
    }

    while (hid_proc_running) {
        hid_dev_event_t dev_evt;
        if (xQueueReceive(hid_event_queue, &dev_evt, pdMS_TO_TICKS(100)) != pdTRUE) {
            // No connect event — check if we need to push a LED update
            if (kb_led_pending.exchange(false) && kb_handle) {
                uint8_t leds = (num_lock_active    ? 0x01 : 0)
                             | (caps_lock_active   ? 0x02 : 0)
                             | (scroll_lock_active ? 0x04 : 0);
                hid_class_request_set_report(kb_handle, HID_REPORT_TYPE_OUTPUT, 0, &leds, 1);
            }
            continue;
        }
        if (dev_evt.event == HID_HOST_DRIVER_EVENT_CONNECTED) {
            hid_host_dev_params_t params;
            if (hid_host_device_get_params(dev_evt.handle, &params) != ESP_OK) {
                continue;
            }
            // Skip non-keyboard/non-mouse HID interfaces (consumer control, system control,
            // vendor-specific, etc.) — opening and starting them causes ESP_ERR_INVALID_STATE
            // errors because we don't configure them for boot protocol before starting transfers.
            if (params.proto != HID_PROTOCOL_KEYBOARD && params.proto != HID_PROTOCOL_MOUSE) {
                LOG_D(TAG, "Ignoring HID interface with unhandled proto=%d", params.proto);
                continue;
            }
            LOG_I(TAG, "HID device connected (proto=%d)", params.proto);

            const hid_host_device_config_t dev_cfg = {
                .callback = hid_interface_callback,
                .callback_arg = nullptr,
            };
            if (hid_host_device_open(dev_evt.handle, &dev_cfg) != ESP_OK) {
                LOG_W(TAG, "hid_host_device_open failed");
                continue;
            }
            // Request boot protocol — many modern devices don't advertise
            // HID_SUBCLASS_BOOT_INTERFACE but still support it.
            hid_class_request_set_protocol(dev_evt.handle, HID_REPORT_PROTOCOL_BOOT);
            if (params.proto == HID_PROTOCOL_KEYBOARD) {
                hid_class_request_set_idle(dev_evt.handle, 0, 0);
                // Give the device time to finish processing the class requests before
                // starting interrupt transfers — some keyboards (e.g. HP KU-0316) NAK
                // the first interrupt IN if we start too soon after SET_PROTOCOL.
                vTaskDelay(pdMS_TO_TICKS(200));
                // Store handle for LED updates
                kb_handle = dev_evt.handle;
                // Push current lock LED state to the keyboard
                uint8_t leds = (num_lock_active    ? 0x01 : 0)
                             | (caps_lock_active   ? 0x02 : 0)
                             | (scroll_lock_active ? 0x04 : 0);
                hid_class_request_set_report(dev_evt.handle, HID_REPORT_TYPE_OUTPUT, 0, &leds, 1);
                // Re-register keyboard indev so hardware_keyboard_is_available() returns true
                // and the pending keyboard group (if any) is applied to kb_indev.
                if (kb_indev && lvgl_port_lock(0)) {
                    tt::lvgl::hardware_keyboard_set_indev(kb_indev);
                    lvgl_port_unlock();
                }
            } else if (params.proto == HID_PROTOCOL_MOUSE) {
                mouse_connected = true;
                if (mouse_cursor && lvgl_port_lock(0)) {
                    lv_obj_remove_flag(mouse_cursor, LV_OBJ_FLAG_HIDDEN);
                    lvgl_port_unlock();
                }
            }
            hid_host_device_start(dev_evt.handle);
        }
    }

    // Clean up LVGL input devices on the way out
    if (lvgl_port_lock(0)) {
        if (mouse_indev) { lv_indev_delete(mouse_indev); mouse_indev = nullptr; }
        if (mouse_cursor) { lv_obj_delete(mouse_cursor); mouse_cursor = nullptr; }
        if (kb_indev)    {
            tt::lvgl::hardware_keyboard_set_indev(nullptr);
            lv_indev_delete(kb_indev);
            kb_indev = nullptr;
        }
        lvgl_port_unlock();
    }

    LOG_I(TAG, "HID proc task stopped");
    xSemaphoreGive(hid_proc_task_done);
    vTaskDelete(nullptr);
}

// ---- HID driver callback: called from HID bg task, enqueue to proc task ----
static void hid_driver_callback(hid_host_device_handle_t handle,
                                const hid_host_driver_event_t event,
                                void* arg)
{
    hid_dev_event_t evt = { handle, event, arg };
    if (hid_event_queue) {
        xQueueSend(hid_event_queue, &evt, 0);
    }
}

extern "C" {

static error_t start() {
    if (hid_proc_task_handle != nullptr) {
        LOG_W(TAG, "HID module already running");
        return ERROR_NONE;
    }

    key_queue = xQueueCreate(KEY_QUEUE_SIZE, sizeof(key_event_t));
    if (!key_queue) {
        LOG_E(TAG, "Failed to create key queue");
        return ERROR_RESOURCE;
    }

    hid_event_queue = xQueueCreate(HID_EVENT_QUEUE_SIZE, sizeof(hid_dev_event_t));
    if (!hid_event_queue) {
        LOG_E(TAG, "Failed to create HID event queue");
        vQueueDelete(key_queue);
        key_queue = nullptr;
        return ERROR_RESOURCE;
    }

    hid_proc_task_done = xSemaphoreCreateBinary();
    if (!hid_proc_task_done) {
        LOG_E(TAG, "Failed to create task done semaphore");
        vQueueDelete(key_queue);
        vQueueDelete(hid_event_queue);
        key_queue = nullptr;
        hid_event_queue = nullptr;
        return ERROR_RESOURCE;
    }

    const hid_host_driver_config_t hid_cfg = {
        .create_background_task = true,
        .task_priority = HID_PROC_TASK_PRIORITY,
        .stack_size = HID_PROC_TASK_STACK,
        .core_id = tskNO_AFFINITY,
        .callback = hid_driver_callback,
        .callback_arg = nullptr,
    };

    if (hid_host_install(&hid_cfg) != ESP_OK) {
        LOG_E(TAG, "hid_host_install failed");
        vQueueDelete(key_queue);
        vQueueDelete(hid_event_queue);
        vSemaphoreDelete(hid_proc_task_done);
        key_queue = nullptr;
        hid_event_queue = nullptr;
        hid_proc_task_done = nullptr;
        return ERROR_RESOURCE;
    }

    hid_proc_running = true;
    BaseType_t result = xTaskCreate(hidProcTask, "hid_proc", HID_PROC_TASK_STACK, nullptr, HID_PROC_TASK_PRIORITY, &hid_proc_task_handle);
    if (result != pdPASS) {
        LOG_E(TAG, "Failed to create hid_proc task");
        hid_proc_running = false;
        hid_host_uninstall();
        vQueueDelete(key_queue);
        vQueueDelete(hid_event_queue);
        vSemaphoreDelete(hid_proc_task_done);
        key_queue = nullptr;
        hid_event_queue = nullptr;
        hid_proc_task_done = nullptr;
        return ERROR_RESOURCE;
    }

    LOG_I(TAG, "USB HID started");
    return ERROR_NONE;
}

static error_t stop() {
    if (hid_proc_task_handle == nullptr) {
        return ERROR_NONE;
    }

    hid_proc_running = false;

    if (xSemaphoreTake(hid_proc_task_done, pdMS_TO_TICKS(HID_STOP_TIMEOUT_MS)) != pdTRUE) {
        LOG_W(TAG, "HID proc task stop timed out, force terminating task");
        vTaskDelete(hid_proc_task_handle);
    }
    hid_proc_task_handle = nullptr;
    vSemaphoreDelete(hid_proc_task_done);
    hid_proc_task_done = nullptr;

    hid_host_uninstall();

    if (key_queue) {
        vQueueDelete(key_queue);
        key_queue = nullptr;
    }
    if (hid_event_queue) {
        vQueueDelete(hid_event_queue);
        hid_event_queue = nullptr;
    }

    LOG_I(TAG, "USB HID stopped");
    return ERROR_NONE;
}

Module usb_host_hid_module = {
    .name = "usb-host-hid",
    .start = start,
    .stop = stop,
    .symbols = nullptr,
    .internal = nullptr,
};

} // extern "C"
