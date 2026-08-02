// SPDX-License-Identifier: Apache-2.0
#include <luavgl/object_private.h>

extern "C" {
#include <lauxlib.h>
}

/**
 * Repeating and one-shot timers: `lvgl.Timer{ period = 500, callback = fn }`.
 *
 * A timer outliving its runtime is the sharpest lifetime hazard in the whole binding - it
 * fires on the LVGL task with no widget to anchor it, so nothing else would notice the
 * runtime had gone. Every timer this runtime creates is therefore tracked and deleted by
 * luavgl_detach_all(); a script that forgets to stop one cannot leave it running.
 *
 * The callback runs directly from the timer rather than through lv_async_call: LVGL timers
 * already run outside event dispatch, so a handler may safely delete widgets. The same
 * caveat as everywhere else still applies - it runs on the LVGL task, holding the LVGL
 * lock, so anything that closes an app must hand off to another thread.
 */

namespace {

constexpr auto* TIMER_METATABLE = "lvgl.Timer";

/** Registry key for the list of live timers, so shutdown can delete them all. */
char TIMERS_KEY = 0;

struct TimerHandle {
    lv_timer_t* timer;
    lua_State* state;
    /** Registry reference to the Lua callback, or LUA_NOREF once the timer is gone. */
    int callback_ref;
    /** Firings left, or -1 for unlimited. LVGL has no getter, so it is tracked here. */
    int32_t remaining;
};

/** Pushes the table of live timers, keyed by lv_timer_t*. */
void push_timers(lua_State* state) {
    lua_pushlightuserdata(state, &TIMERS_KEY);
    if (lua_rawget(state, LUA_REGISTRYINDEX) != LUA_TTABLE) {
        lua_pop(state, 1);
        lua_newtable(state);

        lua_pushlightuserdata(state, &TIMERS_KEY);
        lua_pushvalue(state, -2);
        lua_rawset(state, LUA_REGISTRYINDEX);
    }
}

/** Frees the timer and its callback reference. Safe to call more than once. */
void destroy_timer(TimerHandle* handle) {
    if (handle->timer == nullptr) {
        return;
    }

    push_timers(handle->state);
    lua_pushlightuserdata(handle->state, handle->timer);
    lua_pushnil(handle->state);
    lua_rawset(handle->state, -3);
    lua_pop(handle->state, 1);

    lv_timer_delete(handle->timer);
    handle->timer = nullptr;

    if (handle->callback_ref != LUA_NOREF) {
        luaL_unref(handle->state, LUA_REGISTRYINDEX, handle->callback_ref);
        handle->callback_ref = LUA_NOREF;
    }
}

void on_timer(lv_timer_t* timer) {
    auto* handle = static_cast<TimerHandle*>(lv_timer_get_user_data(timer));
    if (handle == nullptr || handle->timer == nullptr || handle->callback_ref == LUA_NOREF) {
        return;
    }

    lua_State* state = handle->state;

    lua_rawgeti(state, LUA_REGISTRYINDEX, handle->callback_ref);
    if (lua_type(state, -1) != LUA_TFUNCTION) {
        lua_pop(state, 1);
        return;
    }

    // Auto-delete is off (see timer_create), so a finite timer is retired here instead.
    // Done before the callback runs, since the callback may delete the timer itself - and
    // the callback is already on the stack, so unreffing it now is safe.
    if (handle->remaining > 0 && --handle->remaining == 0) {
        destroy_timer(handle);
    }

    // Nothing may touch the state afterwards - the callback may have closed the app
    lua_pcall(state, 0, 0, 0);
}

TimerHandle* check_timer(lua_State* state, int index) {
    return static_cast<TimerHandle*>(luaL_checkudata(state, index, TIMER_METATABLE));
}

/** Returns the live timer, raising if it has already been deleted. */
lv_timer_t* check_live_timer(lua_State* state, int index) {
    auto* handle = check_timer(state, index);
    if (handle->timer == nullptr) {
        luaL_error(state, "timer has been deleted");
    }
    return handle->timer;
}

// Methods ---------------------------------------------------------------------------------

int timer_delete(lua_State* state) {
    destroy_timer(check_timer(state, 1));
    return 0;
}

int timer_pause(lua_State* state) {
    lv_timer_pause(check_live_timer(state, 1));
    lua_settop(state, 1);
    return 1;
}

int timer_resume(lua_State* state) {
    lv_timer_resume(check_live_timer(state, 1));
    lua_settop(state, 1);
    return 1;
}

int timer_set_period(lua_State* state) {
    auto* timer = check_live_timer(state, 1);
    const lua_Integer period = luaL_checkinteger(state, 2);
    if (period <= 0) {
        return luaL_error(state, "period must be positive");
    }

    lv_timer_set_period(timer, static_cast<uint32_t>(period));
    lua_settop(state, 1);
    return 1;
}

/** Makes the timer fire on the next cycle regardless of how much of its period is left. */
int timer_ready(lua_State* state) {
    lv_timer_ready(check_live_timer(state, 1));
    lua_settop(state, 1);
    return 1;
}

int timer_is_valid(lua_State* state) {
    lua_pushboolean(state, check_timer(state, 1)->timer != nullptr);
    return 1;
}

int timer_gc(lua_State* state) {
    // Collecting the handle does NOT stop the timer: a script that starts a repeating
    // timer and keeps no reference still expects it to run. Timers are stopped explicitly,
    // or by luavgl_detach_all() at shutdown.
    (void)state;
    return 0;
}

int timer_tostring(lua_State* state) {
    auto* handle = check_timer(state, 1);
    if (handle->timer == nullptr) {
        lua_pushstring(state, "lvgl.Timer (deleted)");
    } else {
        lua_pushfstring(state, "lvgl.Timer (%p)", handle->timer);
    }
    return 1;
}

const luaL_Reg timer_methods[] = {
    { "delete", timer_delete },
    { "pause", timer_pause },
    { "resume", timer_resume },
    { "set_period", timer_set_period },
    { "ready", timer_ready },
    { "is_valid", timer_is_valid },
    { nullptr, nullptr }
};

/**
 * `lvgl.Timer{ period = ms, callback = fn, repeat_count = n, paused = bool }`
 *
 * `repeat_count` defaults to unlimited; 1 makes a one-shot.
 */
int timer_create(lua_State* state) {
    luaL_checktype(state, 1, LUA_TTABLE);

    lua_getfield(state, 1, "period");
    const lua_Integer period = luaL_optinteger(state, -1, 1000);
    lua_pop(state, 1);

    if (period <= 0) {
        return luaL_error(state, "period must be positive");
    }

    lua_getfield(state, 1, "callback");
    if (lua_type(state, -1) != LUA_TFUNCTION) {
        return luaL_error(state, "a callback function is required");
    }
    const int callback_ref = luaL_ref(state, LUA_REGISTRYINDEX); // pops the function

    auto* handle = static_cast<TimerHandle*>(lua_newuserdatauv(state, sizeof(TimerHandle), 0));
    handle->timer = nullptr;
    handle->state = state;
    handle->callback_ref = callback_ref;
    handle->remaining = -1; // unlimited unless repeat_count says otherwise
    luaL_setmetatable(state, TIMER_METATABLE);

    auto* timer = lv_timer_create(on_timer, static_cast<uint32_t>(period), handle);
    if (timer == nullptr) {
        luaL_unref(state, LUA_REGISTRYINDEX, callback_ref);
        handle->callback_ref = LUA_NOREF;
        return luaL_error(state, "failed to create timer");
    }
    handle->timer = timer;

    // LVGL defaults to auto_delete, freeing the timer itself once repeat_count runs out -
    // which would leave this handle, and the shutdown list, pointing at freed memory.
    // Ownership stays here instead: on_timer() retires an exhausted timer explicitly.
    lv_timer_set_auto_delete(timer, false);

    lua_getfield(state, 1, "repeat_count");
    if (!lua_isnil(state, -1)) {
        const auto count = static_cast<int32_t>(luaL_checkinteger(state, -1));
        handle->remaining = count;
        lv_timer_set_repeat_count(timer, count);
    }
    lua_pop(state, 1);

    lua_getfield(state, 1, "paused");
    if (lua_toboolean(state, -1)) {
        lv_timer_pause(timer);
    }
    lua_pop(state, 1);

    // Remembered so shutdown can delete it even if the script drops the handle
    push_timers(state);
    lua_pushlightuserdata(state, timer);
    lua_pushvalue(state, -3); // the handle userdata
    lua_rawset(state, -3);
    lua_pop(state, 1);

    return 1;
}

} // namespace

void luavgl_open_timer(lua_State* state) {
    luaL_newmetatable(state, TIMER_METATABLE);

    luaL_newlib(state, timer_methods);
    lua_setfield(state, -2, "__index");

    lua_pushcfunction(state, timer_gc);
    lua_setfield(state, -2, "__gc");

    lua_pushcfunction(state, timer_tostring);
    lua_setfield(state, -2, "__tostring");

    lua_pushboolean(state, 0);
    lua_setfield(state, -2, "__metatable");

    lua_pop(state, 1);

    lua_pushcfunction(state, timer_create);
    lua_setfield(state, -2, "Timer");
}

void luavgl_delete_all_timers(lua_State* state) {
    push_timers(state);

    // Collect first: destroy_timer() mutates the table, and mutating during lua_next is
    // undefined.
    lv_timer_t* timers[32];
    size_t count = 0;

    lua_pushnil(state);
    while (lua_next(state, -2) != 0) {
        lua_pop(state, 1); // the handle; the key is the timer pointer

        if (count < sizeof(timers) / sizeof(timers[0])) {
            timers[count++] = static_cast<lv_timer_t*>(lua_touserdata(state, -1));
        }
    }
    lua_pop(state, 1);

    for (size_t i = 0; i < count; i++) {
        push_timers(state);
        lua_pushlightuserdata(state, timers[i]);
        lua_rawget(state, -2);

        if (auto* handle = static_cast<TimerHandle*>(lua_touserdata(state, -1))) {
            destroy_timer(handle);
        }
        lua_pop(state, 2);
    }
}
