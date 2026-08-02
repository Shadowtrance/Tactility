// SPDX-License-Identifier: Apache-2.0
#include <luavgl/object_private.h>

#include <lvgl/widgets/toolbar.h>

extern "C" {
#include <lauxlib.h>
}

/**
 * Binds `lvgl-module`'s toolbar rather than rebuilding one in Lua.
 *
 * A hand-rolled Lua toolbar would look approximately right and behave differently -
 * wrong height for the display density, no themed styling, and crucially no working
 * back navigation, since closing an app goes through the loader rather than through
 * anything a script can reach. Wrapping the real widget means Lua apps get the same
 * toolbar as C++ apps, including whatever it gains later.
 */

namespace {

/** Registry key for the table of nav callbacks, keyed by toolbar object. */
char NAV_CALLBACKS_KEY = 0;

void push_nav_callbacks(lua_State* state) {
    lua_pushlightuserdata(state, &NAV_CALLBACKS_KEY);
    lua_rawget(state, LUA_REGISTRYINDEX);

    if (!lua_istable(state, -1)) {
        lua_pop(state, 1);
        lua_newtable(state);

        lua_pushlightuserdata(state, &NAV_CALLBACKS_KEY);
        lua_pushvalue(state, -2);
        lua_rawset(state, LUA_REGISTRYINDEX);
    }
}

/**
 * Runs on the LVGL task when a toolbar action is pressed.
 *
 * The Lua state arrives as the event's user data. A script error here has nowhere to
 * propagate - LVGL called us, not Lua - so it is swallowed rather than left to unwind
 * through C.
 *
 * **Stopping an app from a handler is asynchronous.** `tt_app_stop()` only posts to the
 * loader's dispatcher thread, so this callback returns first and the app is torn down
 * afterwards - which also means the widget stays alive and clickable throughout. Each
 * callback is therefore one-shot: see the removal below.
 *
 * The stack is balanced before the call rather than after, since the handler may have
 * started tearing this state down by the time it returns.
 */
/** Carries a pending handler from the LVGL event to the async call that runs it. */
struct PendingAction {
    lua_State* state;
    lv_obj_t* target;
    /** Cleared by luavgl_detach_event_targets() so a queued action becomes a no-op. */
    bool cancelled;
    PendingAction* next;
};

/**
 * Every queued action, so shutdown can cancel them.
 *
 * lv_async_call_cancel() matches on both callback *and* user data, and the user data here
 * is a fresh allocation per press, so it cannot be used to cancel "everything for this
 * state". The queued call is left to run instead and told to do nothing.
 */
PendingAction* pending_actions = nullptr;

/**
 * Runs the Lua handler, once the LVGL event that triggered it has fully unwound.
 *
 * See on_action() for why this cannot happen inline.
 */
void run_pending_action(void* context) {
    auto* pending = static_cast<PendingAction*>(context);

    // Unlink first, so the list never holds a freed entry
    for (PendingAction** link = &pending_actions; *link != nullptr; link = &(*link)->next) {
        if (*link == pending) {
            *link = pending->next;
            break;
        }
    }

    const bool cancelled = pending->cancelled;
    lua_State* state = pending->state;
    lv_obj_t* target = pending->target;
    lv_free(pending);

    if (cancelled) {
        return; // the runtime went away between the press and now
    }

    push_nav_callbacks(state);

    lua_pushlightuserdata(state, target);
    if (lua_rawget(state, -2) != LUA_TFUNCTION) {
        lua_pop(state, 2);
        return;
    }

    // One-shot: remove before running. Stopping an app is asynchronous, so the widget
    // stays alive and clickable throughout the teardown, and a second press must not
    // re-enter a runtime that is being destroyed.
    lua_pushlightuserdata(state, target);
    lua_pushnil(state);
    lua_rawset(state, -4);

    lua_insert(state, -2);
    lua_pop(state, 1); // the table, leaving the function

    // Nothing may touch the state afterwards: the handler may have begun teardown.
    lua_pcall(state, 0, 0, 0);
}

void on_action(lv_event_t* event) {
    auto* state = static_cast<lua_State*>(lv_event_get_user_data(event));
    auto* target = static_cast<lv_obj_t*>(lv_event_get_current_target(event));

    auto* pending = static_cast<PendingAction*>(lv_malloc(sizeof(PendingAction)));
    if (pending == nullptr) {
        return;
    }

    pending->state = state;
    pending->target = target;
    pending->cancelled = false;
    pending->next = pending_actions;
    pending_actions = pending;

    // Deferred to the next LVGL timer cycle so the handler does not run inside the event
    // dispatch itself - a handler that deletes the widget it was invoked from would
    // otherwise pull the ground out from under LVGL's event loop.
    //
    // Note this does NOT escape the LVGL lock: lvgl_port_task holds it across the whole
    // of lv_timer_handler(), and lv_async_call runs from a timer. Anything that needs the
    // lock released - notably closing the app, since GuiService::hideApp() takes it -
    // must be deferred by the *handler* onto another thread. See the app's
    // tactility.app.stop() for how.
    lv_async_call(run_pending_action, pending);
}

/**
 * Finds the descendant of `root` carrying our action callback for `state`.
 *
 * The toolbar installs nav actions on an internal child it does not expose, so the widget
 * the callback actually lives on has to be discovered rather than assumed.
 */
lv_obj_t* find_event_target(lv_obj_t* root, lua_State* state) {
    const uint32_t event_count = lv_obj_get_event_count(root);
    for (uint32_t i = 0; i < event_count; i++) {
        lv_event_dsc_t* descriptor = lv_obj_get_event_dsc(root, i);
        if (lv_event_dsc_get_cb(descriptor) == on_action &&
            lv_event_dsc_get_user_data(descriptor) == state) {
            return root;
        }
    }

    const uint32_t child_count = lv_obj_get_child_count(root);
    for (uint32_t i = 0; i < child_count; i++) {
        auto* child = lv_obj_get_child(root, static_cast<int32_t>(i));
        if (auto* found = find_event_target(child, state)) {
            return found;
        }
    }

    return nullptr;
}

/**
 * Removes whatever SHORT_CLICKED handler the toolbar's nav button already carries.
 *
 * That is the firmware's default close action, installed by lvgl_toolbar_create(). It has
 * to go before a script's handler is added, or both run.
 */
void remove_existing_nav_action(lv_obj_t* toolbar) {
    // The nav button is the first descendant with a SHORT_CLICKED callback; action buttons
    // are added later, so at set_nav_action() time this finds the close button.
    lv_obj_t* button = nullptr;

    const uint32_t child_count = lv_obj_get_child_count(toolbar);
    for (uint32_t i = 0; i < child_count && button == nullptr; i++) {
        auto* wrapper = lv_obj_get_child(toolbar, static_cast<int32_t>(i));

        const uint32_t inner_count = lv_obj_get_child_count(wrapper);
        for (uint32_t j = 0; j < inner_count; j++) {
            auto* candidate = lv_obj_get_child(wrapper, static_cast<int32_t>(j));
            if (lv_obj_get_event_count(candidate) > 0) {
                button = candidate;
                break;
            }
        }
    }

    if (button == nullptr) {
        return;
    }

    // Walk backwards: removing by index shifts the ones after it
    for (int32_t i = static_cast<int32_t>(lv_obj_get_event_count(button)) - 1; i >= 0; i--) {
        lv_event_dsc_t* descriptor = lv_obj_get_event_dsc(button, static_cast<uint32_t>(i));
        if (lv_event_dsc_get_cb(descriptor) != on_action) {
            lv_obj_remove_event(button, static_cast<uint32_t>(i));
        }
    }
}

/** Stores `function_index` as the callback for `object`, tracking it for detach. */
void store_callback(lua_State* state, lv_obj_t* object, int function_index) {
    push_nav_callbacks(state);

    lua_pushlightuserdata(state, object);
    lua_pushvalue(state, function_index);
    lua_rawset(state, -3);

    lua_pop(state, 1);

    // The widget the callback lives on can outlive this runtime - a toolbar belongs to
    // the app's screen, which the loader tears down after the app is destroyed - so the
    // object is remembered for luavgl_detach_all() to unhook later.
    luavgl_track_event_target(state, object);
}

int toolbar_create(lua_State* state) {
    lv_obj_t* parent = nullptr;
    int title_index = 1;

    if (luaL_testudata(state, 1, LUAVGL_OBJECT_METATABLE) != nullptr) {
        parent = luavgl_check_object(state, 1);
        title_index = 2;
    } else {
        parent = luavgl_get_root(state);
    }

    const char* title = luaL_checkstring(state, title_index);

    auto* toolbar = lvgl_toolbar_create(parent, title);
    if (toolbar == nullptr) {
        return luaL_error(state, "failed to create toolbar");
    }

    luavgl_push_object(state, toolbar);
    return 1;
}

int toolbar_set_title(lua_State* state) {
    auto* toolbar = luavgl_check_object(state, 1);
    lvgl_toolbar_set_title(toolbar, luaL_checkstring(state, 2));
    lua_settop(state, 1);
    return 1;
}

/**
 * Replaces the leading navigation action.
 *
 * Rarely needed: a fresh toolbar already has a working close button that returns to the
 * launcher, so an app only calls this to do something else on back.
 *
 * Called as `set_nav_action(toolbar, callback)` or, to change the icon too,
 * `set_nav_action(toolbar, icon, callback)`. The icon is an LVGL symbol string such as
 * `lvgl.SYMBOL.CLOSE` - an arbitrary string is treated as an image path and silently
 * renders as nothing, which is exactly the mistake this signature avoids.
 */
int toolbar_set_nav_action(lua_State* state) {
    auto* toolbar = luavgl_check_object(state, 1);

    const char* icon = LV_SYMBOL_CLOSE;
    int callback_index = 2;

    if (lua_isstring(state, 2)) {
        icon = lua_tostring(state, 2);
        callback_index = 3;
    }

    luaL_checktype(state, callback_index, LUA_TFUNCTION);

    // lvgl_toolbar_set_nav_action() *adds* an event callback rather than replacing one,
    // and a fresh toolbar already carries the default close handler. Without removing it
    // first, both fire: the default closes the app and the script's handler then runs
    // too, closing whatever ended up on top afterwards.
    //
    // The default is registered on the same internal button, so it is found the same way
    // - by looking for the widget that carries a SHORT_CLICKED callback.
    remove_existing_nav_action(toolbar);

    lvgl_toolbar_set_nav_action(toolbar, icon, on_action, state);

    // lvgl_toolbar_set_nav_action installs the callback on the toolbar's internal close
    // button, not on the toolbar itself, and does not return it. The callback is keyed by
    // the widget that dispatches it, so the right one has to be found rather than assumed
    // - keying against the toolbar would simply never match at dispatch time.
    lv_obj_t* target = find_event_target(toolbar, state);
    if (target == nullptr) {
        return luaL_error(state, "could not locate the toolbar's nav button");
    }

    store_callback(state, target, callback_index);

    // Track the toolbar too: detach walks each tracked widget's subtree, so tracking the
    // root guarantees every internal child is reached whatever the toolbar does with them.
    luavgl_track_event_target(state, toolbar);

    lua_settop(state, 1);
    return 1;
}

int toolbar_add_text_action(lua_State* state) {
    auto* toolbar = luavgl_check_object(state, 1);
    const char* text = luaL_checkstring(state, 2);
    luaL_checktype(state, 3, LUA_TFUNCTION);

    auto* button = lvgl_toolbar_add_text_button_action(toolbar, text, on_action, state);
    if (button == nullptr) {
        return luaL_error(state, "failed to add toolbar action");
    }

    // Keyed by the button, since each action needs its own callback
    store_callback(state, button, 3);

    // As above: tracking the toolbar means detach sweeps its whole subtree
    luavgl_track_event_target(state, toolbar);

    luavgl_push_object(state, button);
    return 1;
}

int toolbar_add_spinner(lua_State* state) {
    auto* toolbar = luavgl_check_object(state, 1);

    auto* spinner = lvgl_toolbar_add_spinner_action(toolbar);
    if (spinner == nullptr) {
        return luaL_error(state, "failed to add spinner");
    }

    luavgl_push_object(state, spinner);
    return 1;
}

int toolbar_clear_actions(lua_State* state) {
    lvgl_toolbar_clear_actions(luavgl_check_object(state, 1));
    lua_settop(state, 1);
    return 1;
}

/**
 * Removes every one of this state's action callbacks from a widget and its descendants.
 *
 * Recursive because the toolbar puts callbacks on internal children it never exposes: a
 * nav action lands on its close button, an added action on a button inside a wrapper.
 * Detaching only the tracked widget therefore leaves live callbacks behind, pointing at a
 * lua_State that is about to be freed - which is what kept crashing on device.
 *
 * Matches on both the callback and the user data, so a widget shared with another runtime
 * keeps that runtime's callbacks.
 */
void detach_one(lua_State* state, lv_obj_t* object) {
    lv_obj_remove_event_cb_with_user_data(object, on_action, state);

    // Widget handlers registered through obj:on() land on the same tracked list
    luavgl_detach_event_handlers(state, object);

    const uint32_t child_count = lv_obj_get_child_count(object);
    for (uint32_t i = 0; i < child_count; i++) {
        detach_one(state, lv_obj_get_child(object, static_cast<int32_t>(i)));
    }
}


const luaL_Reg toolbar_functions[] = {
    { "create", toolbar_create },
    { "set_title", toolbar_set_title },
    { "set_nav_action", toolbar_set_nav_action },
    { "add_text_action", toolbar_add_text_action },
    { "add_spinner", toolbar_add_spinner },
    { "clear_actions", toolbar_clear_actions },
    { nullptr, nullptr }
};

} // namespace

void luavgl_open_toolbar(lua_State* state) {
    luaL_newlib(state, toolbar_functions);
    lua_setfield(state, -2, "toolbar"); // lvgl.toolbar
}

void luavgl_detach_event_targets(lua_State* state) {
    luavgl_for_each_event_target(state, detach_one);

    // A press can leave a handler queued that has not run yet - the app is closing, after
    // all, and that press is usually what started the teardown. The queued call cannot be
    // cancelled (lv_async_call_cancel matches on the per-press user data), so it is marked
    // instead and will return without touching this state.
    for (PendingAction* pending = pending_actions; pending != nullptr; pending = pending->next) {
        if (pending->state == state) {
            pending->cancelled = true;
        }
    }
}

bool luavgl_test_has_event_callback(lv_obj_t* root, lua_State* state) {
    return find_event_target(root, state) != nullptr;
}

lv_obj_t* luavgl_test_find_event_target(lv_obj_t* root, lua_State* state) {
    return find_event_target(root, state);
}
