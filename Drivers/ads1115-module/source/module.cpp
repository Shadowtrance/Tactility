// SPDX-License-Identifier: Apache-2.0
#include <tactility/driver.h>
#include <tactility/module.h>

extern "C" {

extern Driver ads1115_driver;

static Driver* const ads1115_drivers[] = {
    &ads1115_driver,
    nullptr
};

Module ads1115_module = {
    .name = "ads1115",
    .start = nullptr,
    .stop = nullptr,
    .drivers = ads1115_drivers,
    .symbols = nullptr,
    .internal = nullptr
};

} // extern "C"
