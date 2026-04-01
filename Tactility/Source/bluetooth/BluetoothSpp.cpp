#ifdef ESP_PLATFORM
#include <sdkconfig.h>
#endif

#if defined(CONFIG_BT_NIMBLE_ENABLED)

#include <Tactility/bluetooth/Bluetooth.h>
#include <Tactility/bluetooth/BluetoothSettings.h>

#include <tactility/drivers/bluetooth_serial.h>

namespace tt::bluetooth {

bool sppStart() {
    struct Device* dev = bluetooth_serial_get_device();
    if (dev == nullptr) return false;
    if (bluetooth_serial_start(dev) != ERROR_NONE) return false;
    settings::setSppAutoStart(true);
    return true;
}

void sppStop() {
    struct Device* dev = bluetooth_serial_get_device();
    if (dev == nullptr) return;
    settings::setSppAutoStart(false);
    bluetooth_serial_stop(dev);
}

} // namespace tt::bluetooth

#endif // CONFIG_BT_NIMBLE_ENABLED
