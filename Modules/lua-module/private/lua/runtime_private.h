// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stddef.h>

extern "C" {
#include <lua.h>
}

/** Cap on a stored error message. Lua tracebacks are longer than this, so they get cut. */
constexpr size_t ERROR_MESSAGE_CAPACITY = 512;

/**
 * The definition behind the opaque LuaRuntime in <lua/runtime.h>.
 *
 * Private so that <lua.h> stays out of the public headers: callers get an opaque handle,
 * while the binding sources inside this module can reach the lua_State directly.
 */
struct LuaRuntime {
    lua_State* state = nullptr;
    /** Bytes currently held by the Lua heap, tracked by the runtime's allocator. */
    size_t memory_used = 0;
    char error_message[ERROR_MESSAGE_CAPACITY] = {};
};
