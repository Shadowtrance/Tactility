// SPDX-License-Identifier: Apache-2.0
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

struct Lp5814Config {
    uint8_t address;
    // Bit mask of OUT0 to OUT3
    uint8_t channels;
    // Maximum output current: 51 mA when true, 25.5 mA when false
    bool high_current;
    // Output current per enabled output in 1/255 of the maximum current: 0.2 mA steps at 51 mA, 0.1 mA steps at 25.5 mA
    uint8_t dot_current;
    // Default PWM duty in 1/255
    uint8_t brightness_default;
};

#ifdef __cplusplus
}
#endif
