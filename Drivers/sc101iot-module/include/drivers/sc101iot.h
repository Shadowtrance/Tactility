// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <tactility/drivers/camera.h>
#include <tactility/drivers/gpio.h>
#include <tactility/error.h>

struct Device;

#ifdef __cplusplus
extern "C" {
#endif

struct Sc101iotConfig {
    /** SCCB I2C address (0x68) */
    uint8_t address;

    /** Clockwise mounting correction applied after the requested display rotation. */
    uint16_t rotation_offset;

    /** Mirror the image vertically (module mounting dependent). */
    bool mirror_y;

    /** Output frame size. Both must be non-zero and match the display orientation. */
    uint16_t output_width;
    uint16_t output_height;

    /** DVP master clock frequency. Zero when the module provides its own oscillator. */
    uint32_t xclk_frequency_hz;

    struct GpioPinSpec pin_d0;
    struct GpioPinSpec pin_d1;
    struct GpioPinSpec pin_d2;
    struct GpioPinSpec pin_d3;
    struct GpioPinSpec pin_d4;
    struct GpioPinSpec pin_d5;
    struct GpioPinSpec pin_d6;
    struct GpioPinSpec pin_d7;
    struct GpioPinSpec pin_vsync;
    struct GpioPinSpec pin_de;
    struct GpioPinSpec pin_pclk;
    struct GpioPinSpec pin_xclk;
    struct GpioPinSpec pin_reset;
    /** Sensor power-down pin. Driven low while the camera is open, high otherwise. */
    struct GpioPinSpec pin_pwdn;
    /** Flash LED control pin. Active-low, driven as an open-drain output. */
    struct GpioPinSpec pin_flash;
};

#ifdef __cplusplus
}
#endif
