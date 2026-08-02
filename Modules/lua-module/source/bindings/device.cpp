// SPDX-License-Identifier: Apache-2.0
#include "bindings_private.h"

#include <cassert>
#include <tactility/device.h>

extern "C" {
#include <lauxlib.h>
}

namespace {

/** Metatable name for the device handle userdata. */
constexpr auto* DEVICE_HANDLE_METATABLE = "tactility.device";

/**
 * A borrowed reference to a Device.
 *
 * The kernel hands out references with device_get() and expects exactly one matching
 * device_put(). Lua has no idea about that, so the reference lives in a userdata whose
 * __gc releases it: whether the script calls release() or simply drops the value and
 * lets the collector take it, the count balances. `released` guards against the double
 * put that an explicit release() followed by collection would otherwise cause.
 */
struct DeviceHandle {
    Device* device;
    bool released;
};

DeviceHandle* check_handle(lua_State* state, int index) {
    return static_cast<DeviceHandle*>(luaL_checkudata(state, index, DEVICE_HANDLE_METATABLE));
}

/** Returns the live device, or raises a Lua error if the handle was already released. */
Device* check_live_device(lua_State* state, int index) {
    auto* handle = check_handle(state, index);
    if (handle->released) {
        luaL_error(state, "device handle has already been released");
    }
    return handle->device;
}

/** Wraps an already-referenced device, transferring ownership of that reference to Lua. */
void push_handle(lua_State* state, Device* device) {
    auto* handle = static_cast<DeviceHandle*>(lua_newuserdatauv(state, sizeof(DeviceHandle), 0));
    handle->device = device;
    handle->released = false;
    luaL_setmetatable(state, DEVICE_HANDLE_METATABLE);
}

/** Releases the reference, if it has not been released already. */
void release_handle(DeviceHandle* handle) {
    if (!handle->released) {
        device_put(handle->device);
        handle->released = true;
    }
}

// Lookup ------------------------------------------------------------------------------

/**
 * The lookup functions return nil rather than raising on "not found": a script asking
 * whether a device exists is doing something normal, not something exceptional. A second
 * return value carries the reason so a caller can tell "absent" from "present but not
 * started".
 */
int push_lookup_result(lua_State* state, error_t error, Device* device) {
    if (error == ERROR_NONE) {
        // A success with no device would mean a reference was taken that nothing can
        // release, and every method on the handle would dereference null.
        assert(device != nullptr);
        push_handle(state, device);
        return 1;
    }

    lua_pushnil(state);
    lua_pushstring(state, error_to_string(error));
    return 2;
}

int device_find_by_name(lua_State* state) {
    const char* name = luaL_checkstring(state, 1);

    // The lookup must be sequenced before `device` is read: as arguments to one call, the
    // order is unspecified, and reading it first yields the still-null initial value.
    Device* device = nullptr;
    const error_t error = device_get_by_name(name, &device);

    return push_lookup_result(state, error, device);
}

int device_find_by_compatible(lua_State* state) {
    const char* compatible = luaL_checkstring(state, 1);

    Device* device = nullptr;
    const error_t error = device_get_first_by_compatible(compatible, &device);

    return push_lookup_result(state, error, device);
}

// Iteration ---------------------------------------------------------------------------

/**
 * Collects device names into a table.
 *
 * Names rather than handles, deliberately: taking a reference on every device at once
 * would block any of them from being stopped for as long as the script held the list.
 * A script picks a name from here and looks it up when it actually wants to use one.
 */
bool collect_name(Device* device, void* context) {
    auto* state = static_cast<lua_State*>(context);

    const lua_Integer next = luaL_len(state, -1) + 1;
    lua_pushstring(state, device->name);
    lua_seti(state, -2, next);

    return true;
}

int device_list(lua_State* state) {
    lua_newtable(state);
    device_for_each(state, collect_name);
    return 1;
}

int device_list_children(lua_State* state) {
    auto* device = check_live_device(state, 1);
    lua_newtable(state);
    device_for_each_child(device, state, collect_name);
    return 1;
}

// Handle methods ----------------------------------------------------------------------

int device_name(lua_State* state) {
    lua_pushstring(state, check_live_device(state, 1)->name);
    return 1;
}

int device_is_ready_binding(lua_State* state) {
    lua_pushboolean(state, device_is_ready(check_live_device(state, 1)));
    return 1;
}

int device_is_compatible_binding(lua_State* state) {
    auto* device = check_live_device(state, 1);
    const char* compatible = luaL_checkstring(state, 2);
    lua_pushboolean(state, device_is_compatible(device, compatible));
    return 1;
}

int device_child_count(lua_State* state) {
    lua_pushinteger(state, static_cast<lua_Integer>(device_get_child_count(check_live_device(state, 1))));
    return 1;
}

int device_type_name(lua_State* state) {
    const auto* type = device_get_type(check_live_device(state, 1));
    if (type == nullptr) {
        lua_pushnil(state);
    } else {
        lua_pushstring(state, type->name);
    }
    return 1;
}

int device_parent(lua_State* state) {
    auto* device = check_live_device(state, 1);

    auto* parent = device_get_parent(device);
    if (parent == nullptr) {
        lua_pushnil(state);
        return 1;
    }

    // A fresh reference for the handle we are about to hand out: the parent's lifetime
    // is not implied by the child's, and each handle releases exactly one reference.
    const error_t error = device_get(parent);
    return push_lookup_result(state, error, parent);
}

int device_release(lua_State* state) {
    release_handle(check_handle(state, 1));
    return 0;
}

int device_handle_gc(lua_State* state) {
    release_handle(check_handle(state, 1));
    return 0;
}

int device_handle_tostring(lua_State* state) {
    auto* handle = check_handle(state, 1);
    if (handle->released) {
        lua_pushstring(state, "device (released)");
    } else {
        lua_pushfstring(state, "device (%s)", handle->device->name);
    }
    return 1;
}

const luaL_Reg device_methods[] = {
    { "name", device_name },
    { "type", device_type_name },
    { "is_ready", device_is_ready_binding },
    { "is_compatible", device_is_compatible_binding },
    { "parent", device_parent },
    { "children", device_list_children },
    { "child_count", device_child_count },
    { "release", device_release },
    { nullptr, nullptr }
};

const luaL_Reg device_functions[] = {
    { "list", device_list },
    { "find", device_find_by_name },
    { "find_by_compatible", device_find_by_compatible },
    { nullptr, nullptr }
};

}

void lua_bindings_open_device(lua_State* state) {
    // Metatable for handles, with the methods reachable through __index
    luaL_newmetatable(state, DEVICE_HANDLE_METATABLE);

    luaL_newlib(state, device_methods);
    lua_setfield(state, -2, "__index");

    lua_pushcfunction(state, device_handle_gc);
    lua_setfield(state, -2, "__gc");

    lua_pushcfunction(state, device_handle_tostring);
    lua_setfield(state, -2, "__tostring");

    // Keeps scripts from reaching the metatable and swapping __gc out from under us
    lua_pushboolean(state, 0);
    lua_setfield(state, -2, "__metatable");

    lua_pop(state, 1); // the metatable itself; the registry owns it now

    luaL_newlib(state, device_functions);
    lua_setfield(state, -2, "device"); // tactility.device
}
