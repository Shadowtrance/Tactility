// SPDX-License-Identifier: Apache-2.0
#include "bindings_private.h"

#include <tactility/log.h>

extern "C" {
#include <lauxlib.h>
}

namespace {

/** Default tag for scripts that do not name one, so their output is identifiable. */
constexpr auto* SCRIPT_TAG = "lua-script";

/**
 * Shared body for the five level functions.
 *
 * Accepts either (message) or (tag, message), matching how the C macros read. Values are
 * pushed through luaL_tolstring so that numbers, booleans and tables with __tostring all
 * print, rather than erroring on anything that is not already a string.
 */
int log_at_level(lua_State* state, LogLevel level) {
    const char* tag = SCRIPT_TAG;
    int message_index = 1;

    if (lua_gettop(state) >= 2) {
        tag = luaL_checkstring(state, 1);
        message_index = 2;
    }

    const char* message = luaL_tolstring(state, message_index, nullptr);

    switch (level) {
        case LOG_LEVEL_ERROR:
            LOG_E(tag, "%s", message);
            break;
        case LOG_LEVEL_WARNING:
            LOG_W(tag, "%s", message);
            break;
        case LOG_LEVEL_INFO:
            LOG_I(tag, "%s", message);
            break;
        case LOG_LEVEL_DEBUG:
            LOG_D(tag, "%s", message);
            break;
        case LOG_LEVEL_VERBOSE:
            LOG_V(tag, "%s", message);
            break;
    }

    lua_pop(state, 1); // the string luaL_tolstring pushed
    return 0;
}

int log_error(lua_State* state) { return log_at_level(state, LOG_LEVEL_ERROR); }
int log_warning(lua_State* state) { return log_at_level(state, LOG_LEVEL_WARNING); }
int log_info(lua_State* state) { return log_at_level(state, LOG_LEVEL_INFO); }
int log_debug(lua_State* state) { return log_at_level(state, LOG_LEVEL_DEBUG); }
int log_verbose(lua_State* state) { return log_at_level(state, LOG_LEVEL_VERBOSE); }

const luaL_Reg log_functions[] = {
    { "error", log_error },
    { "warning", log_warning },
    { "info", log_info },
    { "debug", log_debug },
    { "verbose", log_verbose },
    { nullptr, nullptr }
};

}

void lua_bindings_open_log(lua_State* state) {
    luaL_newlib(state, log_functions);
    lua_setfield(state, -2, "log"); // tactility.log, with the tactility table below us
}
