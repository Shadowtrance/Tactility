// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <tactility/error.h>
#include <tactility/drivers/audio_codec.h>

struct Device;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ES7210 codec device configuration (set from devicetree).
 *
 * The ES7210 is an input-only (microphone ADC) codec. The I2C bus is the
 * device's parent (per i2c-device.yaml), while the I2S controller carrying
 * the (typically 4-slot TDM) audio data is referenced by name.
 */
struct Es7210Config {
    /** I2C address on the bus (typically 0x40) */
    uint8_t address;
    /** Name of the I2S controller device that carries audio data (e.g. "i2s0") */
    const char* i2s_device_name;
    /** Bitmask of microphones to enable, e.g. ES7210_SEL_MIC1 | ES7210_SEL_MIC2 | ... */
    uint8_t mic_selected_mask;
};

#ifdef __cplusplus
}
#endif
