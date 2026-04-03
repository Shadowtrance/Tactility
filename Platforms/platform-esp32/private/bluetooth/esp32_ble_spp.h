#pragma once

#ifdef ESP_PLATFORM
#include <sdkconfig.h>
#endif

#if defined(CONFIG_BT_NIMBLE_ENABLED)

#include <tactility/drivers/bluetooth_serial.h>
#include <tactility/error.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <cstdint>
#include <deque>
#include <vector>

struct BleSppCtx {
    SemaphoreHandle_t                rx_mutex;
    std::deque<std::vector<uint8_t>> rx_queue;
};

struct Device;

bool     ble_spp_get_active(struct Device* device);
void     ble_spp_set_active(struct Device* device, bool v);
uint16_t ble_spp_get_conn_handle(struct Device* device);
void     ble_spp_set_conn_handle(struct Device* device, uint16_t h);

extern const BtSerialApi nimble_serial_api;

#endif // CONFIG_BT_NIMBLE_ENABLED
