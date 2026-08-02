// SPDX-License-Identifier: Apache-2.0
#pragma once

extern "C" {
#include <lua.h>
}

/**
 * Each of these expects the `tactility` table on top of the stack, and adds one
 * sub-table to it, leaving the stack as it found it.
 */

void lua_bindings_open_log(lua_State* state);
void lua_bindings_open_device(lua_State* state);
void lua_bindings_open_fs(lua_State* state);
void lua_bindings_open_settings(lua_State* state);
void lua_bindings_open_time(lua_State* state);
void lua_bindings_open_power(lua_State* state);
void lua_bindings_open_wifi(lua_State* state);
