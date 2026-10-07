// SPDX-License-Identifier: Apache-2.0
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

struct Ads1115Config {
    uint8_t address;
    uint16_t full_scale_mv;
};

#ifdef __cplusplus
}
#endif
