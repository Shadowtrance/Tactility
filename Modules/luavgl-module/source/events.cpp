// SPDX-License-Identifier: Apache-2.0
#include <luavgl/object_private.h>

extern "C" {
#include <lauxlib.h>
}

/**
 * Widget event callbacks: `obj:on(event, handler)`.
 *
 * Unlike the toolbar's nav action, these fire repeatedly - a button is pressed many times
 * over an app's life - so the handler stays registered until the widget dies or the script
 * removes it.
 *
 * Three lifetime hazards, all of which bit during checkpoint 6a and are handled here:
 *
 *  - The handler must not run inside LVGL's event dispatch, because a handler that deletes
 *    its own widget would pull the ground out from under the loop. It is deferred to the
 *    next timer cycle with lv_async_call.
 *  - Deferring means the widget can be deleted between the press and the call, so the
 *    queued entry is validated rather than trusted.
 *  - Deferring also means the *runtime* can be freed in between - the usual close-button
 *    case - so pending entries are cancelled by luavgl_detach_all().
 */

namespace {

/** Registry key for the handler table: registry[EVENTS][widget][code] = function. */
char EVENT_HANDLERS_KEY = 0;

/** Queued dispatches, so shutdown can neutralise them. */
struct PendingEvent {
    lua_State* state;
    lv_obj_t* target;
    lv_event_code_t code;
    /**
     * The key, for LV_EVENT_KEY. Captured at dispatch rather than read when the handler
     * runs: the input device reports whatever key is current, and by then that is a later
     * keypress (or none at all).
     */
    uint32_t key;
    bool cancelled;
    PendingEvent* next;
};

PendingEvent* pending_events = nullptr;

/** Pushes registry[EVENT_HANDLERS_KEY], creating it if needed. */
void push_handlers(lua_State* state) {
    lua_pushlightuserdata(state, &EVENT_HANDLERS_KEY);
    if (lua_rawget(state, LUA_REGISTRYINDEX) != LUA_TTABLE) {
        lua_pop(state, 1);
        lua_newtable(state);

        lua_pushlightuserdata(state, &EVENT_HANDLERS_KEY);
        lua_pushvalue(state, -2);
        lua_rawset(state, LUA_REGISTRYINDEX);
    }
}

/**
 * Pushes the per-widget table of handlers.
 * @param create whether to make one when absent; when false, pushes nil instead
 * @return true if a table was pushed
 */
bool push_widget_handlers(lua_State* state, lv_obj_t* object, bool create) {
    push_handlers(state);

    lua_pushlightuserdata(state, object);
    if (lua_rawget(state, -2) == LUA_TTABLE) {
        lua_remove(state, -2); // drop the outer table
        return true;
    }
    lua_pop(state, 1);

    if (!create) {
        lua_pop(state, 1);
        return false;
    }

    lua_newtable(state);
    lua_pushlightuserdata(state, object);
    lua_pushvalue(state, -2);
    lua_rawset(state, -4);

    lua_remove(state, -2);
    return true;
}

void run_pending_event(void* context);

/**
 * Runs on the LVGL task when the widget receives the event.
 *
 * Only queues: see the file comment for why the handler cannot run here.
 */
void on_widget_event(lv_event_t* event) {
    auto* state = static_cast<lua_State*>(lv_event_get_user_data(event));
    auto* target = static_cast<lv_obj_t*>(lv_event_get_current_target(event));
    const lv_event_code_t code = lv_event_get_code(event);

    auto* pending = static_cast<PendingEvent*>(lv_malloc(sizeof(PendingEvent)));
    if (pending == nullptr) {
        return;
    }

    pending->state = state;
    pending->target = target;
    pending->code = code;
    // Read here, while the event is still the current one
    pending->key = (code == LV_EVENT_KEY) ? lv_indev_get_key(lv_indev_active()) : 0;
    pending->cancelled = false;
    pending->next = pending_events;
    pending_events = pending;

    lv_async_call(run_pending_event, pending);
}

void run_pending_event(void* context) {
    auto* pending = static_cast<PendingEvent*>(context);

    for (PendingEvent** link = &pending_events; *link != nullptr; link = &(*link)->next) {
        if (*link == pending) {
            *link = pending->next;
            break;
        }
    }

    const bool cancelled = pending->cancelled;
    lua_State* state = pending->state;
    lv_obj_t* target = pending->target;
    const lv_event_code_t code = pending->code;
    const uint32_t key = pending->key;
    lv_free(pending);

    if (cancelled) {
        return; // the runtime went away between the event and now
    }

    // The widget can have been deleted in the gap - by an earlier handler in the same
    // batch, say - so it is checked rather than assumed.
    if (!lv_obj_is_valid(target)) {
        return;
    }

    if (!push_widget_handlers(state, target, false)) {
        return;
    }

    lua_pushinteger(state, code);
    if (lua_rawget(state, -2) != LUA_TFUNCTION) {
        lua_pop(state, 2);
        return;
    }

    lua_remove(state, -2); // drop the widget table, leaving the function

    // The handler receives its widget, so one function can serve several widgets
    luavgl_push_object(state, target);

    // Key events carry the key as a second argument. Other events pass only the widget, so
    // a handler written for one event does not have to accept an argument it never uses.
    int argument_count = 1;
    if (code == LV_EVENT_KEY) {
        lua_pushinteger(state, static_cast<lua_Integer>(key));
        argument_count = 2;
    }

    // Nothing may touch the state afterwards: the handler may have begun teardown
    lua_pcall(state, argument_count, 0, 0);
}

// Bindings --------------------------------------------------------------------------------

/**
 * `obj:on(event_code, handler)` - registers a handler, replacing any for the same event.
 *
 * Returns the widget so calls chain.
 */
int object_on(lua_State* state) {
    auto* object = luavgl_check_object(state, 1);
    const auto code = static_cast<lv_event_code_t>(luaL_checkinteger(state, 2));
    luaL_checktype(state, 3, LUA_TFUNCTION);

    push_widget_handlers(state, object, true);

    // Only add the LVGL callback the first time this widget/code pair is registered,
    // or repeated on() calls would stack duplicate dispatches.
    lua_pushinteger(state, code);
    const bool already_registered = lua_rawget(state, -2) != LUA_TNIL;
    lua_pop(state, 1);

    lua_pushinteger(state, code);
    lua_pushvalue(state, 3);
    lua_rawset(state, -3);

    lua_pop(state, 1); // the widget table

    if (!already_registered) {
        lv_obj_add_event_cb(object, on_widget_event, code, state);
        luavgl_track_event_target(state, object);
    }

    lua_settop(state, 1);
    return 1;
}

/** `obj:off(event_code)` - removes the handler for one event. */
int object_off(lua_State* state) {
    auto* object = luavgl_check_object(state, 1);
    const auto code = static_cast<lv_event_code_t>(luaL_checkinteger(state, 2));

    if (push_widget_handlers(state, object, false)) {
        lua_pushinteger(state, code);
        lua_pushnil(state);
        lua_rawset(state, -3);
        lua_pop(state, 1);
    }

    // The LVGL callback stays registered; run_pending_event() finds no handler and does
    // nothing. Removing it would mean matching on the code as well as the callback and
    // user data, and leaving it costs one no-op dispatch.
    lua_settop(state, 1);
    return 1;
}

const luaL_Reg event_methods[] = {
    { "on", object_on },
    { "off", object_off },
    { nullptr, nullptr }
};

struct Constant {
    const char* name;
    lua_Integer value;
};

/** The event codes a script realistically needs. Drawing and internal events are omitted. */
const Constant EVENT_CONSTANTS[] = {
    { "PRESSED", LV_EVENT_PRESSED },
    { "PRESSING", LV_EVENT_PRESSING },
    { "PRESS_LOST", LV_EVENT_PRESS_LOST },
    { "SHORT_CLICKED", LV_EVENT_SHORT_CLICKED },
    { "SINGLE_CLICKED", LV_EVENT_SINGLE_CLICKED },
    { "DOUBLE_CLICKED", LV_EVENT_DOUBLE_CLICKED },
    { "TRIPLE_CLICKED", LV_EVENT_TRIPLE_CLICKED },
    { "LONG_PRESSED", LV_EVENT_LONG_PRESSED },
    { "LONG_PRESSED_REPEAT", LV_EVENT_LONG_PRESSED_REPEAT },
    { "CLICKED", LV_EVENT_CLICKED },
    { "RELEASED", LV_EVENT_RELEASED },
    { "SCROLL_BEGIN", LV_EVENT_SCROLL_BEGIN },
    { "SCROLL_END", LV_EVENT_SCROLL_END },
    { "SCROLL", LV_EVENT_SCROLL },
    { "GESTURE", LV_EVENT_GESTURE },
    { "KEY", LV_EVENT_KEY },
    { "FOCUSED", LV_EVENT_FOCUSED },
    { "DEFOCUSED", LV_EVENT_DEFOCUSED },
    { "VALUE_CHANGED", LV_EVENT_VALUE_CHANGED },
    { "READY", LV_EVENT_READY },
    { "CANCEL", LV_EVENT_CANCEL },
};

/**
 * Key codes, as delivered to an LV_EVENT_KEY handler's second argument.
 *
 * These are LVGL's own control codes, not ASCII - printable characters arrive as their
 * ASCII value, so a script compares against `string.byte('a')` for those and against these
 * for the rest.
 */
const Constant KEY_CONSTANTS[] = {
    { "UP", LV_KEY_UP },
    { "DOWN", LV_KEY_DOWN },
    { "RIGHT", LV_KEY_RIGHT },
    { "LEFT", LV_KEY_LEFT },
    { "ESC", LV_KEY_ESC },
    { "DEL", LV_KEY_DEL },
    { "BACKSPACE", LV_KEY_BACKSPACE },
    { "ENTER", LV_KEY_ENTER },
    { "NEXT", LV_KEY_NEXT },
    { "PREV", LV_KEY_PREV },
    { "HOME", LV_KEY_HOME },
    { "END", LV_KEY_END },
};

} // namespace

void luavgl_add_event_methods(lua_State* state) {
    luaL_setfuncs(state, event_methods, 0);
}

void luavgl_open_events(lua_State* state) {
    lua_createtable(state, 0, sizeof(EVENT_CONSTANTS) / sizeof(Constant));

    for (const auto& entry : EVENT_CONSTANTS) {
        lua_pushinteger(state, entry.value);
        lua_setfield(state, -2, entry.name);
    }

    lua_setfield(state, -2, "EVENT");

    lua_createtable(state, 0, sizeof(KEY_CONSTANTS) / sizeof(Constant));

    for (const auto& entry : KEY_CONSTANTS) {
        lua_pushinteger(state, entry.value);
        lua_setfield(state, -2, entry.name);
    }

    lua_setfield(state, -2, "KEY");
}

void luavgl_cancel_pending_events(lua_State* state) {
    for (PendingEvent* pending = pending_events; pending != nullptr; pending = pending->next) {
        if (pending->state == state) {
            pending->cancelled = true;
        }
    }
}

void luavgl_detach_event_handlers(lua_State* state, lv_obj_t* object) {
    lv_obj_remove_event_cb_with_user_data(object, on_widget_event, state);
}
