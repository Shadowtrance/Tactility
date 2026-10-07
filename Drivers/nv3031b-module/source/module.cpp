// SPDX-License-Identifier: Apache-2.0
#include <tactility/driver.h>
#include <tactility/module.h>

extern "C" {

extern Driver nv3031b_driver;

static Driver* const nv3031b_drivers[] = {
    &nv3031b_driver,
    nullptr
};

Module nv3031b_module = {
    .name = "nv3031b",
    .start = nullptr,
    .stop = nullptr,
    .drivers = nv3031b_drivers,
    .symbols = nullptr,
    .internal = nullptr
};

} // extern "C"
