# usb-host-hid-module

USB HID class driver — routes keyboard and mouse events into LVGL.

## What it does

### Keyboard
- Translates HID boot-protocol reports into LVGL key events
- **Printable characters**: full US QWERTY layout including shifted symbols
- **Caps Lock / Num Lock / Scroll Lock**: toggle state is tracked; LEDs on the keyboard are updated via HID output report on each toggle and on connect
- **Shift**: applies to both letter and symbol keys; interacts correctly with Caps Lock (Shift+CapsLock gives lowercase)
- **Tab / Shift+Tab**: mapped to `LV_KEY_NEXT` / `LV_KEY_PREV` for form focus navigation
- **Arrow keys, Home, End, Delete, Backspace, Enter, Escape**: mapped to corresponding LVGL keys
- **Page Up / Page Down**: each fires 8 `LV_KEY_UP` / `LV_KEY_DOWN` events to scroll the focused widget
- **Numpad**: digits and `.` when Num Lock is on; navigation keys (arrows, Home, End, Del) when off; Enter and arithmetic operators (`+-*/`) always active
- **Ctrl / Alt held**: suppresses printable character output (prevents typing 'c' on Ctrl+C)
- **Software key repeat**: 500 ms initial delay, then ~20 repeats/sec while a key is held
- **Multi-key rollover**: up to 6 simultaneous keys via HID boot protocol
- Non-keyboard HID interfaces (consumer control, system control, vendor-specific) are ignored — only boot-protocol keyboard and mouse interfaces are opened

### Mouse
- Accumulated relative displacement → absolute cursor position, clamped to display bounds
- Cursor image (`TT_ASSETS_UI_CURSOR`, 16×16) shown/hidden on the sys layer on connect/disconnect
- Left button drives `LV_INDEV_STATE_PRESSED` on the pointer indev
- Right button → `LV_KEY_ESC` key event
- Scroll wheel → `LV_KEY_UP` / `LV_KEY_DOWN` key events (if the mouse sends a 4th wheel byte in boot protocol — most modern mice do)
- Cursor position is pre-transformed so it tracks correctly across all four display rotations

### Integration
- Registers `LV_INDEV_TYPE_KEYPAD` and `LV_INDEV_TYPE_POINTER` indevs at startup
- Calls `hardware_keyboard_set_indev` on keyboard connect/disconnect so the software keyboard is shown when no USB keyboard is attached
- Thread-safe: HID bg task writes to atomics and a FreeRTOS queue; LVGL task reads via callbacks only

## Dependencies

```yaml
dependencies:
  - Drivers/usb-host-module
```

Requires `TactilityKernel`, `Tactility`, `usb`, and `esp_lvgl_port` in CMake.

## Mouse widget support

Mouse left-click and cursor movement work universally. The other inputs have limited widget coverage in LVGL:

- **Scroll wheel** (`LV_KEY_UP`/`LV_KEY_DOWN` via the keypad indev): works in widgets that respond to those keys — dropdowns scroll correctly, for example. Most other widgets ignore them.
- **Right click** (`LV_KEY_ESC`): works wherever Escape is meaningful (e.g. closing a focused dropdown). No visible effect in most widgets.
- **Middle click**: not mapped — no standard LVGL action for it.

## Required sdkconfig entries

Devices that include this module need the following entries in their `device.properties` `[sdkconfig]` section. These are required for reliable enumeration of real-world keyboards; without them slow or non-compliant devices (common in generic office keyboards) fail to enumerate.

```ini
CONFIG_USB_HOST_HUBS_SUPPORTED=y
CONFIG_USB_HOST_DEBOUNCE_DELAY_MS=500
CONFIG_USB_HOST_RESET_HOLD_MS=100
CONFIG_USB_HOST_RESET_RECOVERY_MS=100
CONFIG_USB_HOST_SET_ADDR_RECOVERY_MS=500
```

ESP-IDF defaults (`DEBOUNCE=250`, `RESET_HOLD=30`, `RESET_RECOVERY=30`, `SET_ADDR=10`) are too tight for many keyboards and cause `CHECK_SHORT_DEV_DESC` / `CHECK_FULL_DEV_DESC` enumeration failures.

## Other compatibility notes

- **FS/LS HID devices through a USB hub**: rejected by ESP-IDF's hub driver (`IDF-10023` — TT/split-transaction not yet implemented). Direct connection always works.
- **Non-US keyboard layouts**: not supported — the keycode→character table is fixed US QWERTY.
- **Media / function keys (F1–F12, consumer control)**: not mapped — no LVGL widget uses them on embedded targets.
