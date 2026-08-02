// SPDX-License-Identifier: Apache-2.0
#include "bindings_private.h"

#include <lua/bindings.h>
#include <lua/runtime.h>
#include <lua/runtime_private.h>

extern "C" {
#include <lauxlib.h>
}

namespace {

/**
 * Bytes the script's own interpreter is using.
 *
 * Distinct from collectgarbage("count"), which reports Lua's view in kilobytes; this is
 * what the kernel allocator actually handed out, which is the number that matters when
 * budgeting against a device's heap.
 */
int lua_heap_used(lua_State* state) {
    lua_pushinteger(state, static_cast<lua_Integer>(lua_runtime_get_memory_used(
        static_cast<const LuaRuntime*>(lua_touserdata(state, lua_upvalueindex(1))))));
    return 1;
}

}

extern "C" error_t lua_bindings_open(struct LuaRuntime* runtime) {
    auto* state = runtime->state;

    lua_newtable(state);

    // Needs the runtime, which is not reachable from the lua_State, so it travels as an
    // upvalue rather than through a global
    lua_pushlightuserdata(state, runtime);
    lua_pushcclosure(state, lua_heap_used, 1);
    lua_setfield(state, -2, "heap_used");

    lua_bindings_open_log(state);
    lua_bindings_open_device(state);
    lua_bindings_open_fs(state);
    lua_bindings_open_settings(state);
    lua_bindings_open_time(state);
    lua_bindings_open_power(state);
    lua_bindings_open_wifi(state);

    lua_setglobal(state, "tactility");

    return ERROR_NONE;
}
