#ifdef ESP_PLATFORM
#include <sdkconfig.h>
#endif

#if defined(CONFIG_BT_NIMBLE_ENABLED)

#include <Tactility/bluetooth/Bluetooth.h>
#include <Tactility/bluetooth/BluetoothSettings.h>

#include <tactility/drivers/bluetooth_midi.h>

namespace tt::bluetooth {

bool midiStart() {
    struct Device* dev = bluetooth_midi_get_device();
    if (dev == nullptr) return false;
    if (bluetooth_midi_start(dev) != ERROR_NONE) return false;
    settings::setMidiAutoStart(true);
    return true;
}

void midiStop() {
    struct Device* dev = bluetooth_midi_get_device();
    if (dev == nullptr) return;
    settings::setMidiAutoStart(false);
    bluetooth_midi_stop(dev);
}

} // namespace tt::bluetooth

#endif // CONFIG_BT_NIMBLE_ENABLED
