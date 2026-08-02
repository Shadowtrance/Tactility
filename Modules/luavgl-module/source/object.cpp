// SPDX-License-Identifier: Apache-2.0
#include <luavgl/object_private.h>

#include <cstring>

extern "C" {
#include <lauxlib.h>
}

namespace {

/** Registry key for the lv_obj_t* -> handle map. Its address is the key. */
char OBJECT_REGISTRY_KEY = 0;

/**
 * Registry key for the strong list of wrapped objects.
 *
 * The map above holds handles weakly so dropped handles can be collected, which means it
 * cannot be used to find what still needs detaching at shutdown. This list keeps the
 * `lv_obj_t*` addresses instead - light userdata, so it pins nothing.
 */
char TRACKED_OBJECTS_KEY = 0;

/** Registry key for widgets carrying Lua event callbacks. See luavgl_track_event_target. */
char EVENT_TARGETS_KEY = 0;

/** Registry key for the default parent. */
char ROOT_KEY = 0;

uint32_t generation_counter = 0;

/** Pushes the weak-valued table mapping objects to their handles. */
void push_object_registry(lua_State* state) {
    lua_pushlightuserdata(state, &OBJECT_REGISTRY_KEY);
    lua_rawget(state, LUA_REGISTRYINDEX);

    if (!lua_istable(state, -1)) {
        lua_pop(state, 1);

        lua_newtable(state);

        // Weak values: a handle the script has dropped can be collected even though the
        // widget is still alive. Without this the map would pin every handle ever made
        // for the lifetime of the runtime.
        lua_newtable(state);
        lua_pushstring(state, "v");
        lua_setfield(state, -2, "__mode");
        lua_setmetatable(state, -2);

        lua_pushlightuserdata(state, &OBJECT_REGISTRY_KEY);
        lua_pushvalue(state, -2);
        lua_rawset(state, LUA_REGISTRYINDEX);
    }
}

/** Pushes the strong list of `lv_obj_t*` this state has wrapped. */
void push_tracked_objects(lua_State* state) {
    lua_pushlightuserdata(state, &TRACKED_OBJECTS_KEY);
    lua_rawget(state, LUA_REGISTRYINDEX);

    if (!lua_istable(state, -1)) {
        lua_pop(state, 1);

        lua_newtable(state);

        lua_pushlightuserdata(state, &TRACKED_OBJECTS_KEY);
        lua_pushvalue(state, -2);
        lua_rawset(state, LUA_REGISTRYINDEX);
    }
}

/**
 * Runs when LVGL destroys the widget, from inside lv_obj_delete().
 *
 * Nulls the handle so later use raises, and drops the registry entry so the address can
 * be reused safely. The Lua state comes through user_data because LVGL callbacks carry no
 * other context.
 */
void on_object_deleted(lv_event_t* event) {
    auto* state = static_cast<lua_State*>(lv_event_get_user_data(event));
    auto* object = static_cast<lv_obj_t*>(lv_event_get_current_target(event));

    push_object_registry(state);

    lua_pushlightuserdata(state, object);
    lua_rawget(state, -2);

    if (auto* handle = static_cast<ObjectHandle*>(lua_touserdata(state, -1))) {
        handle->object = nullptr;
    }
    lua_pop(state, 1);

    lua_pushlightuserdata(state, object);
    lua_pushnil(state);
    lua_rawset(state, -3);

    lua_pop(state, 1);
}

// Methods -------------------------------------------------------------------------------

int object_set(lua_State* state) {
    auto* object = luavgl_check_object(state, 1);
    luaL_checktype(state, 2, LUA_TTABLE);

    luavgl_apply_properties(state, object, 2);

    lua_settop(state, 1); // return self, so calls can chain
    return 1;
}

int object_delete(lua_State* state) {
    auto* object = luavgl_check_object(state, 1);

    // on_object_deleted() runs synchronously from here and clears the handle
    lv_obj_delete(object);
    return 0;
}

/**
 * `obj:clean()` - deletes every child, keeping the object itself. Returns the object.
 *
 * The natural way to swap views: a container is emptied and rebuilt rather than every
 * widget being tracked and deleted by hand. Handles for the children are invalidated by
 * the same LV_EVENT_DELETE path as delete(), so a script still holding one gets
 * "widget has been deleted" rather than a dangling pointer.
 */
int object_clean(lua_State* state) {
    auto* object = luavgl_check_object(state, 1);

    lv_obj_clean(object);

    lua_settop(state, 1);
    return 1;
}

int object_is_valid(lua_State* state) {
    auto* handle = static_cast<ObjectHandle*>(luaL_checkudata(state, 1, LUAVGL_OBJECT_METATABLE));
    lua_pushboolean(state, handle->object != nullptr);
    return 1;
}

int object_get_parent(lua_State* state) {
    auto* object = luavgl_check_object(state, 1);

    auto* parent = lv_obj_get_parent(object);
    if (parent == nullptr) {
        lua_pushnil(state);
    } else {
        luavgl_push_object(state, parent);
    }
    return 1;
}

int object_get_children(lua_State* state) {
    auto* object = luavgl_check_object(state, 1);

    lua_newtable(state);

    const uint32_t count = lv_obj_get_child_count(object);
    for (uint32_t i = 0; i < count; i++) {
        luavgl_push_object(state, lv_obj_get_child(object, static_cast<int32_t>(i)));
        lua_seti(state, -2, static_cast<lua_Integer>(i + 1));
    }

    return 1;
}

int object_center(lua_State* state) {
    lv_obj_center(luavgl_check_object(state, 1));
    lua_settop(state, 1);
    return 1;
}

/**
 * Forces LVGL to lay the subtree out now.
 *
 * Sizes and positions are not readable until a layout pass has run, so a script that sets
 * a size and immediately reads it back gets 0 without this. LVGL does the pass itself once
 * per frame; this is for scripts that need the geometry before then.
 */
int object_update_layout(lua_State* state) {
    lv_obj_update_layout(luavgl_check_object(state, 1));
    lua_settop(state, 1);
    return 1;
}

int object_get_size(lua_State* state) {
    auto* object = luavgl_check_object(state, 1);
    lua_pushinteger(state, lv_obj_get_width(object));
    lua_pushinteger(state, lv_obj_get_height(object));
    return 2;
}

int object_get_position(lua_State* state) {
    auto* object = luavgl_check_object(state, 1);
    lua_pushinteger(state, lv_obj_get_x(object));
    lua_pushinteger(state, lv_obj_get_y(object));
    return 2;
}

int object_add_flag(lua_State* state) {
    auto* object = luavgl_check_object(state, 1);
    lv_obj_add_flag(object, static_cast<lv_obj_flag_t>(luaL_checkinteger(state, 2)));
    lua_settop(state, 1);
    return 1;
}

int object_remove_flag(lua_State* state) {
    auto* object = luavgl_check_object(state, 1);
    lv_obj_remove_flag(object, static_cast<lv_obj_flag_t>(luaL_checkinteger(state, 2)));
    lua_settop(state, 1);
    return 1;
}

int object_has_flag(lua_State* state) {
    auto* object = luavgl_check_object(state, 1);
    lua_pushboolean(state, lv_obj_has_flag(object, static_cast<lv_obj_flag_t>(luaL_checkinteger(state, 2))));
    return 1;
}

int object_tostring(lua_State* state) {
    auto* handle = static_cast<ObjectHandle*>(luaL_checkudata(state, 1, LUAVGL_OBJECT_METATABLE));

    if (handle->object == nullptr) {
        lua_pushstring(state, "lvgl.Object (deleted)");
    } else {
        lua_pushfstring(state, "lvgl.Object (%p)", handle->object);
    }
    return 1;
}

/** Identity is the underlying widget, so two handles on one widget compare equal. */
int object_equals(lua_State* state) {
    auto* left = static_cast<ObjectHandle*>(luaL_testudata(state, 1, LUAVGL_OBJECT_METATABLE));
    auto* right = static_cast<ObjectHandle*>(luaL_testudata(state, 2, LUAVGL_OBJECT_METATABLE));

    lua_pushboolean(state, left != nullptr && right != nullptr &&
                               left->object != nullptr && left->object == right->object);
    return 1;
}

const luaL_Reg object_methods[] = {
    { "set", object_set },
    { "delete", object_delete },
    { "clean", object_clean },
    { "is_valid", object_is_valid },
    { "parent", object_get_parent },
    { "children", object_get_children },
    { "center", object_center },
    { "update_layout", object_update_layout },
    { "size", object_get_size },
    { "position", object_get_position },
    { "add_flag", object_add_flag },
    { "remove_flag", object_remove_flag },
    { "has_flag", object_has_flag },
    { nullptr, nullptr }
};

} // namespace

uint32_t luavgl_next_generation() {
    return ++generation_counter;
}

void luavgl_track_event_target(lua_State* state, lv_obj_t* object) {
    lua_pushlightuserdata(state, &EVENT_TARGETS_KEY);
    if (lua_rawget(state, LUA_REGISTRYINDEX) != LUA_TTABLE) {
        lua_pop(state, 1);
        lua_newtable(state);

        lua_pushlightuserdata(state, &EVENT_TARGETS_KEY);
        lua_pushvalue(state, -2);
        lua_rawset(state, LUA_REGISTRYINDEX);
    }

    // Keyed by the object so re-registering a callback does not add a duplicate entry
    lua_pushlightuserdata(state, object);
    lua_pushboolean(state, 1);
    lua_rawset(state, -3);

    lua_pop(state, 1);
}

/** Iterates the tracked event targets, calling `visit` for each still-live widget. */
void luavgl_for_each_event_target(lua_State* state, void (*visit)(lua_State*, lv_obj_t*)) {
    lua_pushlightuserdata(state, &EVENT_TARGETS_KEY);
    if (lua_rawget(state, LUA_REGISTRYINDEX) != LUA_TTABLE) {
        lua_pop(state, 1);
        return;
    }
    const int targets_index = lua_gettop(state);

    lua_pushnil(state);
    while (lua_next(state, targets_index) != 0) {
        lua_pop(state, 1); // the value; the key is the object

        auto* object = static_cast<lv_obj_t*>(lua_touserdata(state, -1));
        if (object == nullptr) {
            continue;
        }

        // The tracked set includes widgets LVGL has since freed, and touching one of
        // those is a use-after-free. Ask LVGL directly rather than consulting the object
        // map: that map holds handles *weakly*, so a widget whose Lua handle has been
        // collected looks absent there while still being very much alive - and skipping
        // it would leave its delete callback pointing at a freed lua_State.
        if (lv_obj_is_valid(object)) {
            visit(state, object);
        }
    }

    lua_pop(state, 1); // targets

    lua_pushlightuserdata(state, &EVENT_TARGETS_KEY);
    lua_pushnil(state);
    lua_rawset(state, LUA_REGISTRYINDEX);
}

void luavgl_push_object(lua_State* state, lv_obj_t* object) {
    if (object == nullptr) {
        lua_pushnil(state);
        return;
    }

    push_object_registry(state);

    // Reuse the existing handle when this widget has been wrapped before
    lua_pushlightuserdata(state, object);
    if (lua_rawget(state, -2) == LUA_TUSERDATA) {
        lua_remove(state, -2); // drop the registry, leave the handle
        return;
    }
    lua_pop(state, 1); // the nil

    auto* handle = static_cast<ObjectHandle*>(lua_newuserdatauv(state, sizeof(ObjectHandle), 0));
    handle->object = object;
    handle->generation = luavgl_next_generation();
    luaL_setmetatable(state, LUAVGL_OBJECT_METATABLE);

    // The delete callback is what keeps the handle honest once LVGL frees the widget
    lv_obj_add_event_cb(object, on_object_deleted, LV_EVENT_DELETE, state);

    lua_pushlightuserdata(state, object);
    lua_pushvalue(state, -2);
    lua_rawset(state, -4);

    lua_remove(state, -2); // drop the registry

    // Remember the address so luavgl_detach_all() can remove the callback if the runtime
    // is closed while the widget is still alive
    push_tracked_objects(state);
    lua_pushlightuserdata(state, object);
    lua_rawseti(state, -2, luaL_len(state, -2) + 1);
    lua_pop(state, 1);
}

void luavgl_detach_all(lua_State* state) {
    // Timers first: they have no widget to anchor them, so nothing else would ever notice
    // them still running against a freed state.
    luavgl_delete_all_timers(state);

    // Then any event dispatch already queued for the next LVGL cycle
    luavgl_cancel_pending_events(state);

    // Event callbacks: these sit on widgets this runtime may not own (a toolbar belongs
    // to the app's screen), so they outlive it unless removed here.
    luavgl_detach_event_targets(state);

    push_tracked_objects(state);
    const int tracked_index = lua_gettop(state);

    const lua_Integer count = luaL_len(state, tracked_index);
    for (lua_Integer i = 1; i <= count; i++) {
        lua_rawgeti(state, tracked_index, i);
        auto* object = static_cast<lv_obj_t*>(lua_touserdata(state, -1));
        lua_pop(state, 1);

        if (object == nullptr) {
            continue;
        }

        // The tracked list holds every address ever wrapped, including widgets LVGL has
        // since freed - touching one of those is a use-after-free.
        //
        // LVGL is asked directly rather than the object map being consulted: that map
        // holds handles weakly, so a widget whose Lua handle has been collected looks
        // absent there while still being alive. Trusting it left live widgets with a
        // delete callback pointing at a freed lua_State, which crashed inside
        // lv_obj_clean() when the loader tore the app's screen down.
        if (!lv_obj_is_valid(object)) {
            continue;
        }

        // Matches on both the callback and the user data, so other runtimes' callbacks on
        // the same widget are left alone.
        lv_obj_remove_event_cb_with_user_data(object, on_object_deleted, state);

        // Styles are held by pointer, not copied, so a widget outliving this runtime would
        // dereference freed style memory on its next redraw. Removing them here is what
        // makes freeing the styles below safe.
        luavgl_remove_styles(state, object);
    }

    lua_pop(state, 1); // tracked list

    // Only now that no widget references them
    luavgl_free_all_styles(state);

    // Drop both tables so a reopened binding starts clean
    lua_pushlightuserdata(state, &TRACKED_OBJECTS_KEY);
    lua_pushnil(state);
    lua_rawset(state, LUA_REGISTRYINDEX);

    lua_pushlightuserdata(state, &OBJECT_REGISTRY_KEY);
    lua_pushnil(state);
    lua_rawset(state, LUA_REGISTRYINDEX);
}

lv_obj_t* luavgl_check_object(lua_State* state, int index) {
    auto* handle = static_cast<ObjectHandle*>(luaL_checkudata(state, index, LUAVGL_OBJECT_METATABLE));

    if (handle->object == nullptr) {
        luaL_error(state, "widget has been deleted");
    }

    return handle->object;
}

lv_obj_t* luavgl_get_root(lua_State* state) {
    lua_pushlightuserdata(state, &ROOT_KEY);
    lua_rawget(state, LUA_REGISTRYINDEX);

    auto* root = static_cast<lv_obj_t*>(lua_touserdata(state, -1));
    lua_pop(state, 1);

    // No explicit root: fall back to the active screen, so a bare script still draws
    return root != nullptr ? root : lv_screen_active();
}

void luavgl_set_root(lua_State* state, lv_obj_t* root) {
    lua_pushlightuserdata(state, &ROOT_KEY);
    lua_pushlightuserdata(state, root);
    lua_rawset(state, LUA_REGISTRYINDEX);
}

int luavgl_create_widget(lua_State* state, WidgetCreate create) {
    // Argument may be a parent handle, a property table, both, or neither
    lv_obj_t* parent = nullptr;
    int properties_index = 0;

    if (luaL_testudata(state, 1, LUAVGL_OBJECT_METATABLE) != nullptr) {
        parent = luavgl_check_object(state, 1);
        if (lua_istable(state, 2)) {
            properties_index = 2;
        }
    } else if (lua_istable(state, 1)) {
        properties_index = 1;
    }

    if (parent == nullptr) {
        parent = luavgl_get_root(state);
    }

    auto* object = create(parent);
    if (object == nullptr) {
        return luaL_error(state, "failed to create widget");
    }

    luavgl_push_object(state, object);

    if (properties_index != 0) {
        luavgl_apply_properties(state, object, properties_index);
    }

    return 1;
}

void luavgl_open_object(lua_State* state) {
    luaL_newmetatable(state, LUAVGL_OBJECT_METATABLE);

    luaL_newlib(state, object_methods);
    luavgl_add_event_methods(state);  // on()/off(), in events.cpp
    luavgl_add_widget_methods(state); // value()/text()/checked, in widgets.cpp
    luavgl_add_style_methods(state);  // add_style()/remove_style(), in style.cpp
    lua_setfield(state, -2, "__index");

    lua_pushcfunction(state, object_tostring);
    lua_setfield(state, -2, "__tostring");

    lua_pushcfunction(state, object_equals);
    lua_setfield(state, -2, "__eq");

    // No __gc: LVGL owns the widget, and collecting a handle must not delete it. A script
    // that wants the widget gone calls delete() explicitly, exactly as in C.

    lua_pushboolean(state, 0);
    lua_setfield(state, -2, "__metatable");

    lua_pop(state, 1);
}
