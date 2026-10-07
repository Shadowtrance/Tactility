// SPDX-License-Identifier: Apache-2.0
#include <tactility/driver.h>
#include <tactility/module.h>

extern "C" {

extern Driver es7243e_driver;

static Driver* const es7243e_drivers[] = {
    &es7243e_driver,
    nullptr
};

Module es7243e_module = {
    .name = "es7243e",
    .start = nullptr,
    .stop = nullptr,
    .drivers = es7243e_drivers,
    .symbols = nullptr,
    .internal = nullptr
};

} // extern "C"
