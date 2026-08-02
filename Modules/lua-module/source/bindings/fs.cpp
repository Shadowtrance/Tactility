// SPDX-License-Identifier: Apache-2.0
#include "bindings_private.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include <tactility/filesystem/file_mutex.h>
#include <tactility/filesystem/file_system.h>
#include <tactility/paths.h>

extern "C" {
#include <lauxlib.h>
}

namespace {

/**
 * Every entry point here takes the mount's file mutex for the duration of the operation.
 *
 * That is the whole reason this binding exists rather than leaving scripts to Lua's own
 * `io`: on boards where the SD card shares a bus with the display, the mutex is what
 * keeps a file read from interleaving with a display transfer. `io.open` knows nothing
 * about it. See Tactility/Source/file/FileMutexLvgl.cpp for where the mutexes come from.
 *
 * A path with no registered mount resolves to an all-null mutex, which the file_mutex_*
 * calls treat as a no-op, so this is safe for internal flash too.
 */
struct ScopedFileLock {
    FileMutex mutex = {};

    explicit ScopedFileLock(const char* path) {
        file_mutex_get(&mutex, path);
        file_mutex_lock(&mutex);
    }

    ~ScopedFileLock() { file_mutex_unlock(&mutex); }

    ScopedFileLock(const ScopedFileLock&) = delete;
    ScopedFileLock& operator=(const ScopedFileLock&) = delete;
};

/** Pushes nil plus a message, the convention for "failed but not exceptional". */
int push_failure(lua_State* state, const char* message) {
    lua_pushnil(state);
    lua_pushstring(state, message);
    return 2;
}

int push_errno_failure(lua_State* state) {
    return push_failure(state, std::strerror(errno));
}

// Reading and writing -----------------------------------------------------------------

/**
 * Reads a whole file and returns it as a string, or nil plus a reason.
 *
 * Whole-file only, deliberately: a script holding a FILE* across yields would hold the
 * mount's mutex with it, which on a shared bus stalls the display. Streaming would need
 * a handle type with its own lifetime rules - worth adding only if something needs it.
 */
int fs_read(lua_State* state) {
    const char* path = luaL_checkstring(state, 1);

    const ScopedFileLock lock(path);

    auto* file = std::fopen(path, "rb");
    if (file == nullptr) {
        return push_errno_failure(state);
    }

    if (std::fseek(file, 0, SEEK_END) != 0) {
        std::fclose(file);
        return push_errno_failure(state);
    }

    const long size = std::ftell(file);
    if (size < 0) {
        std::fclose(file);
        return push_errno_failure(state);
    }
    std::rewind(file);

    // luaL_Buffer allocates through the Lua allocator, so a large file counts against the
    // runtime's memory accounting rather than silently using the system heap.
    luaL_Buffer buffer;
    char* target = luaL_buffinitsize(state, &buffer, static_cast<size_t>(size));

    const size_t read = std::fread(target, 1, static_cast<size_t>(size), file);
    std::fclose(file);

    if (read != static_cast<size_t>(size)) {
        // The buffer is still on the stack and must be resolved before pushing a result.
        luaL_pushresultsize(&buffer, 0);
        lua_pop(state, 1);
        return push_failure(state, "short read");
    }

    luaL_pushresultsize(&buffer, read);
    return 1;
}

/** Shared body for write (truncate) and append. */
int write_with_mode(lua_State* state, const char* mode) {
    const char* path = luaL_checkstring(state, 1);

    size_t length = 0;
    const char* content = luaL_checklstring(state, 2, &length);

    const ScopedFileLock lock(path);

    auto* file = std::fopen(path, mode);
    if (file == nullptr) {
        return push_errno_failure(state);
    }

    const size_t written = std::fwrite(content, 1, length, file);

    // fclose can fail on a flush, which is how a full SD card usually surfaces - so the
    // result matters as much as fwrite's.
    const bool closed = std::fclose(file) == 0;

    if (written != length || !closed) {
        return push_failure(state, "write failed");
    }

    lua_pushboolean(state, 1);
    return 1;
}

int fs_write(lua_State* state) { return write_with_mode(state, "wb"); }
int fs_append(lua_State* state) { return write_with_mode(state, "ab"); }

// Inspection --------------------------------------------------------------------------

/** Fills `out` for path, returning false if it cannot be stat'ed. */
bool stat_path(const char* path, struct stat& out) {
    const ScopedFileLock lock(path);
    return ::stat(path, &out) == 0;
}

int fs_exists(lua_State* state) {
    struct stat status = {};
    lua_pushboolean(state, stat_path(luaL_checkstring(state, 1), status));
    return 1;
}

int fs_is_file(lua_State* state) {
    struct stat status = {};
    const bool found = stat_path(luaL_checkstring(state, 1), status);
    lua_pushboolean(state, found && S_ISREG(status.st_mode));
    return 1;
}

int fs_is_directory(lua_State* state) {
    struct stat status = {};
    const bool found = stat_path(luaL_checkstring(state, 1), status);
    lua_pushboolean(state, found && S_ISDIR(status.st_mode));
    return 1;
}

int fs_size(lua_State* state) {
    struct stat status = {};
    if (!stat_path(luaL_checkstring(state, 1), status)) {
        return push_failure(state, "not found");
    }
    lua_pushinteger(state, static_cast<lua_Integer>(status.st_size));
    return 1;
}

// Directories -------------------------------------------------------------------------

/**
 * Lists a directory as an array of { name, directory } tables.
 *
 * "." and ".." are skipped: every caller has to filter them out, and a script that
 * recurses without noticing them loops forever.
 */
int fs_list(lua_State* state) {
    const char* path = luaL_checkstring(state, 1);

    const ScopedFileLock lock(path);

    auto* directory = ::opendir(path);
    if (directory == nullptr) {
        return push_errno_failure(state);
    }

    lua_newtable(state);

    lua_Integer index = 1;
    while (auto* entry = ::readdir(directory)) {
        if (std::strcmp(entry->d_name, ".") == 0 || std::strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        lua_newtable(state);

        lua_pushstring(state, entry->d_name);
        lua_setfield(state, -2, "name");

        // d_type is not filled in by every filesystem; DT_UNKNOWN means "ask stat", but
        // that would be a stat per entry, so report false and let the caller check.
        lua_pushboolean(state, entry->d_type == DT_DIR);
        lua_setfield(state, -2, "directory");

        lua_seti(state, -2, index++);
    }

    ::closedir(directory);
    return 1;
}

int fs_make_directory(lua_State* state) {
    const char* path = luaL_checkstring(state, 1);

    const ScopedFileLock lock(path);

    if (::mkdir(path, 0777) != 0 && errno != EEXIST) {
        return push_errno_failure(state);
    }

    lua_pushboolean(state, 1);
    return 1;
}

/** Removes a file or an empty directory. Recursive removal is left to the script. */
int fs_remove(lua_State* state) {
    const char* path = luaL_checkstring(state, 1);

    struct stat status = {};
    const ScopedFileLock lock(path);

    if (::stat(path, &status) != 0) {
        return push_errno_failure(state);
    }

    const int result = S_ISDIR(status.st_mode) ? ::rmdir(path) : ::unlink(path);
    if (result != 0) {
        return push_errno_failure(state);
    }

    lua_pushboolean(state, 1);
    return 1;
}

// Mounts ------------------------------------------------------------------------------

bool collect_mount(FileSystem* fs, void* context) {
    auto* state = static_cast<lua_State*>(context);

    if (!file_system_is_mounted(fs)) {
        return true;
    }

    char path[128];
    if (file_system_get_path(fs, path, sizeof(path)) != ERROR_NONE) {
        return true;
    }

    const lua_Integer next = luaL_len(state, -1) + 1;
    lua_pushstring(state, path);
    lua_seti(state, -2, next);

    return true;
}

/** The mounted filesystem roots, so a script can find the SD card without guessing. */
int fs_mounts(lua_State* state) {
    lua_newtable(state);
    file_system_for_each(state, collect_mount);
    return 1;
}

/** Where an app should keep data that survives an OS upgrade. */
int fs_user_data_path(lua_State* state) {
    char path[128];
    if (paths_get_user_data_path(path, sizeof(path)) != ERROR_NONE) {
        return push_failure(state, "no user data location available");
    }
    lua_pushstring(state, path);
    return 1;
}

const luaL_Reg fs_functions[] = {
    { "read", fs_read },
    { "write", fs_write },
    { "append", fs_append },
    { "exists", fs_exists },
    { "is_file", fs_is_file },
    { "is_directory", fs_is_directory },
    { "size", fs_size },
    { "list", fs_list },
    { "make_directory", fs_make_directory },
    { "remove", fs_remove },
    { "mounts", fs_mounts },
    { "user_data_path", fs_user_data_path },
    { nullptr, nullptr }
};

}

void lua_bindings_open_fs(lua_State* state) {
    luaL_newlib(state, fs_functions);
    lua_setfield(state, -2, "fs"); // tactility.fs
}
