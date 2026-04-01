#ifdef ESP_PLATFORM
#include <sdkconfig.h>
#endif

#if defined(CONFIG_BT_NIMBLE_ENABLED)

#include <Tactility/bluetooth/Bluetooth.h>

#include <tactility/drivers/bluetooth_hid_device.h>

namespace tt::bluetooth {

bool hidDeviceStart(uint16_t appearance) {
    struct Device* dev = bluetooth_hid_device_get_device();
    if (dev == nullptr) return false;
    BtHidDeviceMode mode;
    switch (appearance) {
        case 0x03C2: mode = BT_HID_DEVICE_MODE_MOUSE;           break;
        case 0x03C4: mode = BT_HID_DEVICE_MODE_GAMEPAD;         break;
        case 0x03C0: mode = BT_HID_DEVICE_MODE_KEYBOARD_MOUSE;  break;
        default:     mode = BT_HID_DEVICE_MODE_KEYBOARD;        break;
    }
    return bluetooth_hid_device_start(dev, mode) == ERROR_NONE;
}

void hidDeviceStop() {
    struct Device* dev = bluetooth_hid_device_get_device();
    if (dev == nullptr) return;
    bluetooth_hid_device_stop(dev);
}

} // namespace tt::bluetooth

#endif // CONFIG_BT_NIMBLE_ENABLED
