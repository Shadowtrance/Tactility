#pragma once

#ifdef ESP_PLATFORM
#include <sdkconfig.h>
#endif

#if defined(CONFIG_BT_NIMBLE_ENABLED)

#include <tactility/drivers/bluetooth_hid_device.h>

#include <atomic>
#include <cstdint>

enum class BleHidProfile { None, KbConsumer, Mouse, KbMouse, Gamepad };

struct BleHidDeviceCtx {
    std::atomic<uint16_t> hid_conn_handle;
};

struct Device;

bool ble_hid_get_active(struct Device* device);
void ble_hid_set_active(struct Device* device, bool v);

// device must be the hid_device child Device*.
void ble_hid_init_gatt();
void ble_hid_init_gatt_handles();
void ble_hid_switch_profile(struct Device* hid_child, BleHidProfile profile);

extern const BtHidDeviceApi nimble_hid_device_api;

#endif // CONFIG_BT_NIMBLE_ENABLED
