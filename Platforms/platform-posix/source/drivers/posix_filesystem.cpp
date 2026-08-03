// SPDX-License-Identifier: Apache-2.0
#include <tactility/filesystem/file_system.h>
#include <tactility/log.h>

#include <sys/stat.h>

#include <cerrno>
#include <cstring>

/**
 * Registers the simulator's data directory as a file system.
 *
 * Without this the simulator has no file systems at all, and anything that walks them finds
 * nothing - most visibly registerInstalledAppsFromFileSystems(), which is why an app placed
 * in data/tactility/app was never picked up at boot.
 *
 * There is nothing to mount: the directory is already there, backed by the host. The API is
 * implemented as a thin wrapper so the rest of the system can treat it like any other mount.
 */

constexpr auto* TAG = "PosixFileSystem";

namespace {

/** Relative, matching file::MOUNT_POINT_DATA on non-ESP builds. */
constexpr auto* DATA_PATH = "data";

error_t data_mount(void*) {
    // Created rather than required: a fresh checkout has no data/ until something writes to
    // it, and failing to boot over a missing directory would be unhelpful.
    if (::mkdir(DATA_PATH, 0777) != 0 && errno != EEXIST) {
        LOG_E(TAG, "Failed to create %s: %s", DATA_PATH, strerror(errno));
        return ERROR_RESOURCE;
    }
    return ERROR_NONE;
}

error_t data_unmount(void*) {
    return ERROR_NONE;
}

bool data_is_mounted(void*) {
    struct stat status = {};
    return ::stat(DATA_PATH, &status) == 0 && S_ISDIR(status.st_mode);
}

error_t data_get_path(void*, char* out_path, size_t out_path_size) {
    const size_t length = std::strlen(DATA_PATH);
    if (out_path_size <= length) {
        return ERROR_INVALID_ARGUMENT;
    }
    std::memcpy(out_path, DATA_PATH, length + 1);
    return ERROR_NONE;
}

const FileSystemApi DATA_FILE_SYSTEM_API = {
    .mount = data_mount,
    .unmount = data_unmount,
    .is_mounted = data_is_mounted,
    .get_path = data_get_path,
};

} // namespace

extern "C" void posix_file_systems_register() {
    if (data_mount(nullptr) != ERROR_NONE) {
        return;
    }

    if (file_system_add(&DATA_FILE_SYSTEM_API, nullptr) == nullptr) {
        LOG_E(TAG, "Failed to register %s", DATA_PATH);
        return;
    }

    LOG_I(TAG, "Registered %s", DATA_PATH);
}
