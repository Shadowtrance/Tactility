// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <tactility/module.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

extern struct Module usb_host_module;

/** Returns true if the USB host library is currently installed and running. */
bool usb_host_is_running(void);

#ifdef __cplusplus
}
#endif
