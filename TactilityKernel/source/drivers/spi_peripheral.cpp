// SPDX-License-Identifier: Apache-2.0
#include <tactility/drivers/spi_peripheral.h>
#include <tactility/driver.h>
#include <tactility/module.h>

extern "C" {

static error_t start(Device*) { return ERROR_NONE; }
static error_t stop(Device*) { return ERROR_NONE; }

const DeviceType SPI_PERIPHERAL_TYPE = {
    .name = "spi_peripheral"
};

extern Module root_module;

Driver spi_peripheral_driver = {
    .name = "spi_peripheral",
    .compatible = (const char*[]) { "spi-peripheral", nullptr },
    .start_device = start,
    .stop_device = stop,
    .api = nullptr,
    .device_type = &SPI_PERIPHERAL_TYPE,
    .owner = &root_module,
    .internal = nullptr
};

}
