#ifdef ESP_PLATFORM
#include <sdkconfig.h>
#endif

#if defined(CONFIG_BT_NIMBLE_ENABLED)

#include <tactility/driver.h>
#include <tactility/drivers/bluetooth.h>

// Forward declare the NimBLE BluetoothApi implemented in Tactility/Source/service/bluetooth/.
// The full implementation lives there because it depends on Tactility-level framework
// utilities (PubSub, RecursiveMutex, LVGL indev). This file provides the driver registration
// entry point so the platform layer owns the "driver is available" declaration.
namespace tt::service::bluetooth {
    extern const BluetoothApi nimble_bluetooth_api;
}

static const char* esp32_bt_compatible[] = { "esp32,ble-nimble", nullptr };

Driver esp32_bluetooth_driver = {
    .name       = "esp32-bluetooth",
    .compatible = esp32_bt_compatible,
    .start_device = nullptr,
    .stop_device  = nullptr,
    .api        = &tt::service::bluetooth::nimble_bluetooth_api,
    .device_type = &BLUETOOTH_TYPE,
    .owner      = nullptr,
    .internal   = nullptr,
};

#endif // CONFIG_BT_NIMBLE_ENABLED
