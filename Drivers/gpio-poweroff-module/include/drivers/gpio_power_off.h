// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stdint.h>

#include <tactility/device.h>
#include <tactility/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Configuration for a whole-device power-off request line.
 *
 * The line is expected to be an open-drain request that is externally pulled up and is only
 * ever driven low (e.g. ESP-Mosaico's POWER_SWITCH signal to its SAM8108 power controller).
 * The driver keeps it high-impedance while idle and switches it to a low output on request.
 */
struct GpioPowerOffConfig {
    /** Shutdown request pin */
    struct GpioPinSpec pin;
    /**
     * How long to keep the line asserted, in milliseconds, before giving up when the power
     * did not actually drop. The line is released again in that case so the device is not
     * left in a half-off state.
     */
    uint32_t assert_timeout_ms;
};

#ifdef __cplusplus
}
#endif
