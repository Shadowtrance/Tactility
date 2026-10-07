// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stdint.h>

struct Device;

#ifdef __cplusplus
extern "C" {
#endif

struct Es7243eConfig {
    /** I2C address on the bus */
    uint8_t address;
    /** I2S controller device that carries audio data */
    struct Device* i2s_device;
    /** Extra digital gain applied by audio_stream as an integer percentage (100 = 1.0x) */
    uint16_t input_gain_percent;
};

#ifdef __cplusplus
}
#endif
