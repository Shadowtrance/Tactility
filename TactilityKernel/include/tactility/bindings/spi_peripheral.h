// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <tactility/bindings/bindings.h>
#include <tactility/drivers/spi_peripheral.h>

#ifdef __cplusplus
extern "C" {
#endif

DEFINE_DEVICETREE(spi_peripheral, struct SpiPeripheralConfig)

#ifdef __cplusplus
}
#endif
