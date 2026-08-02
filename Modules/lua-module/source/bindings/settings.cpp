// SPDX-License-Identifier: Apache-2.0
#include "bindings_private.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include <tactility/filesystem/file_mutex.h>
#include <tactility/service/service_paths.h>

extern "C" {
#include <lauxlib.h>
}

namespace {

/**
 * Key/value persistence, one file per namespace under the service user-data directory.
 *
 * File-backed rather than NVS-backed because `lua-module` is a kernel module: the
 * `Preferences` API that ELF apps reach through `tt_preferences.h` is C++ in the
 * Tactility layer, which the kernel cannot depend on. `gps-module` solves the same
 * problem the same way (see Modules/gps-module/source/gps_settings.cpp), and this
 * inherits the useful property that settings survive an OS upgrade.
 *
 * Format is `key=value` lines. Values are stored escaped so that newlines and the
 * separator survive a round trip - a naive properties file silently corrupts both.
 */

constexpr size_t MAX_KEY_LENGTH = 64;
constexpr size_t MAX_NAMESPACE_LENGTH = 64;
/** Guards against a script filling the filesystem through repeated puts. */
constexpr size_t MAX_FILE_SIZE = 64 * 1024;

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

int push_failure(lua_State* state, const char* message) {
    lua_pushnil(state);
    lua_pushstring(state, message);
    return 2;
}

/**
 * Namespaces and keys become part of a file path, so anything that could escape the
 * directory or produce a surprising filename is rejected rather than sanitised - silently
 * rewriting a name would make two different keys collide.
 */
bool is_valid_name(const char* name, size_t max_length) {
    const size_t length = std::strlen(name);
    if (length == 0 || length > max_length) {
        return false;
    }

    for (const char* c = name; *c != '\0'; c++) {
        const bool allowed = (*c >= 'a' && *c <= 'z') || (*c >= 'A' && *c <= 'Z') ||
                             (*c >= '0' && *c <= '9') || *c == '-' || *c == '_' || *c == '.';
        if (!allowed) {
            return false;
        }
    }

    // ".." would traverse; a leading "." hides the file
    return name[0] != '.';
}

/** Recursively creates every missing component of `path`. Best effort. */
void ensure_directory_exists(const char* path) {
    char buffer[224];
    std::strncpy(buffer, path, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    for (char* p = buffer + 1; *p != '\0'; p++) {
        if (*p == '/') {
            *p = '\0';
            ::mkdir(buffer, 0777);
            *p = '/';
        }
    }
    ::mkdir(buffer, 0777);
}

bool get_storage_path(const char* name_space, char* out_path, size_t out_path_size) {
    char directory[192];
    if (service_paths_get_user_data_directory("lua", directory, sizeof(directory)) != ERROR_NONE) {
        return false;
    }

    ensure_directory_exists(directory);

    const int written = std::snprintf(out_path, out_path_size, "%s/%s.properties", directory, name_space);
    return written > 0 && static_cast<size_t>(written) < out_path_size;
}

// Escaping ----------------------------------------------------------------------------

/** Escapes the characters that would otherwise break the line-based format. */
std::string escape(const char* value, size_t length) {
    std::string result;
    result.reserve(length);

    for (size_t i = 0; i < length; i++) {
        switch (value[i]) {
            case '\\': result += "\\\\"; break;
            case '\n': result += "\\n"; break;
            case '\r': result += "\\r"; break;
            case '\0': result += "\\0"; break;
            default: result += value[i];
        }
    }

    return result;
}

std::string unescape(const std::string& value) {
    std::string result;
    result.reserve(value.size());

    for (size_t i = 0; i < value.size(); i++) {
        if (value[i] != '\\' || i + 1 >= value.size()) {
            result += value[i];
            continue;
        }

        switch (value[++i]) {
            case 'n': result += '\n'; break;
            case 'r': result += '\r'; break;
            case '0': result += '\0'; break;
            case '\\': result += '\\'; break;
            default: result += value[i]; // unknown escape: keep the character as-is
        }
    }

    return result;
}

// Storage -------------------------------------------------------------------------------

using Entries = std::vector<std::pair<std::string, std::string>>;

/** Turns one complete line into an entry, ignoring anything without a separator. */
void append_line(Entries& entries, std::string& text) {
    while (!text.empty() && (text.back() == '\n' || text.back() == '\r')) {
        text.pop_back();
    }

    const size_t separator = text.find('=');
    if (separator != std::string::npos) {
        // Only the first '=' separates; later ones belong to the value
        entries.emplace_back(text.substr(0, separator), unescape(text.substr(separator + 1)));
    }

    text.clear();
}

/**
 * Reads every entry. A missing file is an empty store, not an error.
 *
 * Accumulates into a std::string rather than reading into a fixed line buffer: a value
 * longer than the buffer would otherwise be split across two reads, and the tail - having
 * no '=' in it - would be dropped, silently truncating the value.
 */
Entries load(const char* path) {
    Entries entries;

    auto* file = std::fopen(path, "rb");
    if (file == nullptr) {
        return entries;
    }

    std::string text;
    char chunk[512];

    while (std::fgets(chunk, sizeof(chunk), file) != nullptr) {
        text += chunk;

        // fgets stops at the newline or when the buffer fills; only the former ends a line
        if (text.back() == '\n') {
            append_line(entries, text);
        }
    }

    // A final line with no trailing newline
    if (!text.empty()) {
        append_line(entries, text);
    }

    std::fclose(file);
    return entries;
}

/**
 * Writes every entry, replacing the file.
 *
 * Writes to a temporary and renames, so an interrupted write (power loss, a full card)
 * leaves the previous settings intact rather than a truncated file.
 */
bool store(const char* path, const Entries& entries) {
    const std::string temporary_path = std::string(path) + ".tmp";

    auto* file = std::fopen(temporary_path.c_str(), "wb");
    if (file == nullptr) {
        return false;
    }

    size_t total = 0;
    bool ok = true;

    for (const auto& [key, value] : entries) {
        const std::string escaped = escape(value.c_str(), value.size());

        total += key.size() + escaped.size() + 2;
        if (total > MAX_FILE_SIZE) {
            ok = false;
            break;
        }

        if (std::fprintf(file, "%s=%s\n", key.c_str(), escaped.c_str()) < 0) {
            ok = false;
            break;
        }
    }

    // A failed flush on close is how a full card usually surfaces
    if (std::fclose(file) != 0) {
        ok = false;
    }

    if (!ok) {
        ::unlink(temporary_path.c_str());
        return false;
    }

    // rename() is atomic on the filesystems in use here, so the switch is all-or-nothing
    if (::rename(temporary_path.c_str(), path) != 0) {
        ::unlink(temporary_path.c_str());
        return false;
    }

    return true;
}

// Bindings ------------------------------------------------------------------------------

/** Validates the (namespace, key) arguments and resolves the backing file. */
bool check_arguments(lua_State* state, char* out_path, size_t out_path_size, const char** out_key) {
    const char* name_space = luaL_checkstring(state, 1);
    if (!is_valid_name(name_space, MAX_NAMESPACE_LENGTH)) {
        luaL_error(state, "invalid namespace: use letters, digits, '-', '_' or '.'");
    }

    if (out_key != nullptr) {
        *out_key = luaL_checkstring(state, 2);
        if (!is_valid_name(*out_key, MAX_KEY_LENGTH)) {
            luaL_error(state, "invalid key: use letters, digits, '-', '_' or '.'");
        }
    }

    return get_storage_path(name_space, out_path, out_path_size);
}

/**
 * Returns the stored string, or the given default when absent.
 *
 * Everything is stored as a string; a script that wants a number uses tonumber(). Typed
 * getters would need the type recorded per key, which is not worth it for the handful of
 * values an app keeps.
 */
int settings_get(lua_State* state) {
    char path[256];
    const char* key = nullptr;
    if (!check_arguments(state, path, sizeof(path), &key)) {
        return push_failure(state, "no user data location available");
    }

    const ScopedFileLock lock(path);

    for (const auto& [stored_key, value] : load(path)) {
        if (stored_key == key) {
            lua_pushlstring(state, value.c_str(), value.size());
            return 1;
        }
    }

    // Absent: hand back the default (nil when not given), so callers can write
    //   local n = tonumber(tactility.settings.get("app", "count", "0"))
    if (lua_gettop(state) >= 3) {
        lua_pushvalue(state, 3);
    } else {
        lua_pushnil(state);
    }
    return 1;
}

int settings_set(lua_State* state) {
    char path[256];
    const char* key = nullptr;
    if (!check_arguments(state, path, sizeof(path), &key)) {
        return push_failure(state, "no user data location available");
    }

    size_t length = 0;
    const char* value = luaL_checklstring(state, 3, &length);

    const ScopedFileLock lock(path);

    Entries entries = load(path);

    bool replaced = false;
    for (auto& entry : entries) {
        if (entry.first == key) {
            entry.second.assign(value, length);
            replaced = true;
            break;
        }
    }

    if (!replaced) {
        entries.emplace_back(key, std::string(value, length));
    }

    if (!store(path, entries)) {
        return push_failure(state, "failed to write settings");
    }

    lua_pushboolean(state, 1);
    return 1;
}

int settings_remove(lua_State* state) {
    char path[256];
    const char* key = nullptr;
    if (!check_arguments(state, path, sizeof(path), &key)) {
        return push_failure(state, "no user data location available");
    }

    const ScopedFileLock lock(path);

    Entries entries = load(path);

    bool removed = false;
    for (auto it = entries.begin(); it != entries.end(); ++it) {
        if (it->first == key) {
            entries.erase(it);
            removed = true;
            break;
        }
    }

    if (removed && !store(path, entries)) {
        return push_failure(state, "failed to write settings");
    }

    lua_pushboolean(state, removed);
    return 1;
}

int settings_has(lua_State* state) {
    char path[256];
    const char* key = nullptr;
    if (!check_arguments(state, path, sizeof(path), &key)) {
        return push_failure(state, "no user data location available");
    }

    const ScopedFileLock lock(path);

    for (const auto& [stored_key, value] : load(path)) {
        if (stored_key == key) {
            lua_pushboolean(state, 1);
            return 1;
        }
    }

    lua_pushboolean(state, 0);
    return 1;
}

/** Every key in the namespace, so a script can enumerate what it stored. */
int settings_keys(lua_State* state) {
    char path[256];
    if (!check_arguments(state, path, sizeof(path), nullptr)) {
        return push_failure(state, "no user data location available");
    }

    const ScopedFileLock lock(path);

    lua_newtable(state);

    lua_Integer index = 1;
    for (const auto& [key, value] : load(path)) {
        lua_pushstring(state, key.c_str());
        lua_seti(state, -2, index++);
    }

    return 1;
}

/** Drops the whole namespace. */
int settings_clear(lua_State* state) {
    char path[256];
    if (!check_arguments(state, path, sizeof(path), nullptr)) {
        return push_failure(state, "no user data location available");
    }

    const ScopedFileLock lock(path);

    // Already absent counts as cleared
    if (::unlink(path) != 0) {
        struct stat status = {};
        if (::stat(path, &status) == 0) {
            return push_failure(state, "failed to clear settings");
        }
    }

    lua_pushboolean(state, 1);
    return 1;
}

const luaL_Reg settings_functions[] = {
    { "get", settings_get },
    { "set", settings_set },
    { "remove", settings_remove },
    { "has", settings_has },
    { "keys", settings_keys },
    { "clear", settings_clear },
    { nullptr, nullptr }
};

}

void lua_bindings_open_settings(lua_State* state) {
    luaL_newlib(state, settings_functions);
    lua_setfield(state, -2, "settings"); // tactility.settings
}
