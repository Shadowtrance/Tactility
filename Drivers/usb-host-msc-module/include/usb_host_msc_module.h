// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <tactility/module.h>

#ifdef __cplusplus
extern "C" {
#endif

extern struct Module usb_host_msc_module;

/** Mount path prefix used for USB drives. Drives mount at /usb0, /usb1, etc. */
#define USB_MSC_MOUNT_PATH "/usb"

/**
 * @brief Safely eject a mounted USB drive.
 * Flushes the FAT filesystem, unmounts the VFS, and releases the USB device.
 * After a successful eject the drive can be physically removed without data loss.
 * @param mount_path Full mount path of the drive (e.g. "/usb0").
 * @return true if the drive was found and ejected, false if not found.
 */
bool usb_msc_eject(const char* mount_path);

#ifdef __cplusplus
}
#endif
