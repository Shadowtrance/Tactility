// SPDX-License-Identifier: Apache-2.0
#include <tactility/driver.h>
#include <tactility/module.h>

extern "C" {

extern Driver sc101iot_driver;

static Driver* const sc101iot_drivers[] = {
    &sc101iot_driver,
    nullptr
};

Module sc101iot_module = {
    .name = "sc101iot",
    .start = nullptr,
    .stop = nullptr,
    .drivers = sc101iot_drivers,
    .symbols = nullptr,
    .internal = nullptr
};

} // extern "C"
