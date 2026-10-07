// SPDX-License-Identifier: Apache-2.0
#include <tactility/driver.h>
#include <tactility/module.h>

extern "C" {

extern Driver lp5814_driver;

static Driver* const lp5814_drivers[] = {
    &lp5814_driver,
    nullptr
};

Module lp5814_module = {
    .name = "lp5814",
    .start = nullptr,
    .stop = nullptr,
    .drivers = lp5814_drivers,
    .symbols = nullptr,
    .internal = nullptr
};

} // extern "C"
