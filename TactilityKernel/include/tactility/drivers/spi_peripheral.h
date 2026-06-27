// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stdint.h>
#include <tactility/device.h>

#ifdef __cplusplus
extern "C" {
#endif

struct SpiPeripheralConfig {
    uint8_t _unused;
};

extern const struct DeviceType SPI_PERIPHERAL_TYPE;

#ifdef __cplusplus
}
#endif
