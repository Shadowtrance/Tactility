// SPDX-License-Identifier: Apache-2.0
#include "bindings_private.h"

#include <ctime>

#include <tactility/delay.h>
#include <tactility/time.h>

extern "C" {
#include <lauxlib.h>
}

namespace {

/** Milliseconds since boot. Monotonic, so safe for measuring intervals. */
int time_uptime(lua_State* state) {
    lua_pushinteger(state, static_cast<lua_Integer>(get_millis()));
    return 1;
}

/**
 * Microseconds since boot.
 *
 * On ESP32 LUA_32BITS caps integers at 2^31, which this overflows after ~36 minutes of
 * uptime, so it is pushed as a float. That keeps it monotonic and usable for timing at
 * the cost of precision past 2^24 microseconds - fine for the "how long did that take"
 * case this exists for.
 */
int time_uptime_micros(lua_State* state) {
    lua_pushnumber(state, static_cast<lua_Number>(get_micros_since_boot()));
    return 1;
}

/** Wall-clock seconds since the Unix epoch, or nil if the clock is not set. */
int time_now(lua_State* state) {
    const std::time_t now = std::time(nullptr);

    // Before NTP or an RTC has run, the clock sits at (or near) the epoch. Reporting that
    // as a real time would have scripts computing dates in 1970, so say so instead.
    if (now < 1000000000) { // 2001-09-09, comfortably before any real clock reading
        lua_pushnil(state);
        lua_pushstring(state, "clock not set");
        return 2;
    }

    lua_pushinteger(state, static_cast<lua_Integer>(now));
    return 1;
}

/**
 * Splits a timestamp into a table, in local time.
 *
 * Mirrors os.date("*t") rather than inventing a shape, so scripts and existing Lua code
 * agree. Defaults to now when called with no argument.
 */
int time_date(lua_State* state) {
    std::time_t timestamp;
    if (lua_isnoneornil(state, 1)) {
        timestamp = std::time(nullptr);
    } else {
        timestamp = static_cast<std::time_t>(luaL_checkinteger(state, 1));
    }

    std::tm broken_down = {};
    if (localtime_r(&timestamp, &broken_down) == nullptr) {
        lua_pushnil(state);
        lua_pushstring(state, "invalid timestamp");
        return 2;
    }

    lua_newtable(state);

    const auto set = [state](const char* key, lua_Integer value) {
        lua_pushinteger(state, value);
        lua_setfield(state, -2, key);
    };

    set("year", broken_down.tm_year + 1900);
    set("month", broken_down.tm_mon + 1);  // tm_mon is 0-11; Lua convention is 1-12
    set("day", broken_down.tm_mday);
    set("hour", broken_down.tm_hour);
    set("min", broken_down.tm_min);
    set("sec", broken_down.tm_sec);
    set("wday", broken_down.tm_wday + 1);  // tm_wday is 0-6; os.date uses 1-7
    set("yday", broken_down.tm_yday + 1);

    lua_pushboolean(state, broken_down.tm_isdst > 0);
    lua_setfield(state, -2, "isdst");

    return 1;
}

/**
 * Blocks the calling thread.
 *
 * This blocks whatever task the script runs on. Once Lua apps drive the UI that will be
 * the LVGL task, and sleeping there freezes the display for the duration - so this is for
 * short waits, not for pacing an app. Timers are the right tool for that, and are not
 * bound yet.
 */
int time_sleep(lua_State* state) {
    const lua_Integer milliseconds = luaL_checkinteger(state, 1);
    if (milliseconds < 0) {
        return luaL_error(state, "sleep duration must not be negative");
    }

    delay_millis(static_cast<uint32_t>(milliseconds));
    return 0;
}

const luaL_Reg time_functions[] = {
    { "uptime", time_uptime },
    { "uptime_micros", time_uptime_micros },
    { "now", time_now },
    { "date", time_date },
    { "sleep", time_sleep },
    { nullptr, nullptr }
};

}

void lua_bindings_open_time(lua_State* state) {
    luaL_newlib(state, time_functions);
    lua_setfield(state, -2, "time"); // tactility.time
}
