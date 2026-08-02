// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <lvgl.h>

extern "C" {
#include <lua.h>
}

/** Metatable name shared by every widget handle. */
inline constexpr auto* LUAVGL_OBJECT_METATABLE = "lvgl.Object";

/**
 * A Lua handle on an `lv_obj_t`.
 *
 * LVGL owns object lifetime and deletes children with their parent, which Lua's collector
 * knows nothing about. Two mechanisms keep the two in step:
 *
 *  - An `LV_EVENT_DELETE` callback nulls `object` when LVGL destroys the widget, so a
 *    handle the script still holds raises "deleted" instead of dereferencing freed memory.
 *  - The registry maps `lv_obj_t*` to its handle, so wrapping the same widget twice
 *    returns the same userdata and `a == b` behaves.
 *
 * `generation` closes the gap luavgl documents but does not solve: LVGL can free an
 * object and allocate a new one at the same address before Lua has collected the old
 * handle, which would make a stale handle silently address the new widget. Every wrap
 * takes the next generation, and a handle only resolves while its generation matches the
 * one recorded for that address.
 */
struct ObjectHandle {
    lv_obj_t* object;
    uint32_t generation;
};

/** Monotonic counter behind ObjectHandle::generation. */
uint32_t luavgl_next_generation();

/**
 * Pushes the handle for `object`, creating it if this is the first time it is seen.
 * Returns the same userdata for the same live object.
 */
void luavgl_push_object(lua_State* state, lv_obj_t* object);

/** Returns the live object, raising a Lua error if the handle is stale. */
lv_obj_t* luavgl_check_object(lua_State* state, int index);

/** Registers the shared object metatable and its methods. */
void luavgl_open_object(lua_State* state);

/** Adds the `toolbar` sub-table to the `lvgl` table on top of the stack. */
void luavgl_open_toolbar(lua_State* state);

/** Adds on()/off() to the object method table on top of the stack. */
void luavgl_add_event_methods(lua_State* state);

/** Adds value()/text()/is_checked()/set_checked() to the object method table on top. */
void luavgl_add_widget_methods(lua_State* state);

/** Adds the widget constructors to the `lvgl` table on top of the stack. */
void luavgl_open_widgets(lua_State* state);

/** Adds the `EVENT` constants to the `lvgl` table on top of the stack. */
void luavgl_open_events(lua_State* state);

/** Adds the `Timer` constructor and its metatable. `lvgl` table must be on top. */
void luavgl_open_timer(lua_State* state);

/** Adds the `Style` constructor and its metatable. `lvgl` table must be on top. */
void luavgl_open_style(lua_State* state);

/** Adds add_style()/remove_style() to the object method table on top of the stack. */
void luavgl_add_style_methods(lua_State* state);

/** Returns the style behind a `lvgl.Style` handle, raising if the value is not one. */
lv_style_t* luavgl_check_style(lua_State* state, int index);

/**
 * Resolves a font name ("small"/"default"/"large"/"icon") to a face, raising on anything
 * else.
 *
 * Names rather than pointers: `lvgl-module` picks the face per display density, so a
 * script asking for "large" stays legible on a 320x240 panel and a 1280x720 one. Exposing
 * raw `lv_font_t*` would tie a script to one device.
 */
const lv_font_t* luavgl_check_font(lua_State* state, int index);

/**
 * Removes every style this runtime created from `object`.
 *
 * Styles are held by pointer, not copied, so a widget outliving the runtime would
 * dereference freed style memory on its next redraw. Called from luavgl_detach_all() for
 * each live widget, before luavgl_free_all_styles().
 */
void luavgl_remove_styles(lua_State* state, lv_obj_t* object);

/**
 * Frees every style this runtime created.
 *
 * Only safe once luavgl_remove_styles() has run for every widget that might reference one.
 */
void luavgl_free_all_styles(lua_State* state);

/** Neutralises queued event dispatches for this state. Called from luavgl_detach_all(). */
void luavgl_cancel_pending_events(lua_State* state);

/** Removes this state's event callbacks from one widget. */
void luavgl_detach_event_handlers(lua_State* state, struct _lv_obj_t* object);

/**
 * Deletes every timer this runtime created.
 *
 * A timer has no widget to anchor it, so nothing else would notice its runtime had gone -
 * it would simply keep firing into freed memory. Called from luavgl_detach_all().
 */
void luavgl_delete_all_timers(lua_State* state);

/**
 * Remembers a widget carrying a Lua event callback, so luavgl_detach_all() can unhook it.
 *
 * Needed for callbacks installed on widgets the runtime does not own - a toolbar belongs
 * to the app's screen, which the loader tears down after the app is destroyed, by which
 * point the callback would point at a freed lua_State (and, for a side-loaded app, an
 * unloaded ELF).
 */
void luavgl_track_event_target(lua_State* state, struct _lv_obj_t* object);

/**
 * Removes the event callbacks tracked above. Implemented in toolbar.cpp, where the
 * callback function itself lives, and called from luavgl_detach_all().
 */
void luavgl_detach_event_targets(lua_State* state);

/** Visits each tracked event target that is still alive, then forgets them all. */
void luavgl_for_each_event_target(lua_State* state, void (*visit)(lua_State*, struct _lv_obj_t*));

/**
 * Whether any widget in `root`'s subtree still carries an event callback for `state`.
 * Exposed for tests: the callbacks live on internal children the bindings do not return,
 * so checking only the widget a binding handed back proves nothing.
 */
bool luavgl_test_has_event_callback(struct _lv_obj_t* root, lua_State* state);

/** The widget in `root`'s subtree that dispatch would deliver to, or NULL. Tests only. */
struct _lv_obj_t* luavgl_test_find_event_target(struct _lv_obj_t* root, lua_State* state);

/** The root new widgets parent to when the script gives none. */
lv_obj_t* luavgl_get_root(lua_State* state);
void luavgl_set_root(lua_State* state, lv_obj_t* root);

/**
 * Detaches every delete callback this state installed.
 *
 * Must run before the lua_State is closed. Widgets routinely outlive the runtime that
 * created them - an app closes its Lua state while its LVGL tree is still on screen, and
 * the tree is torn down afterwards. Without this, the eventual LV_EVENT_DELETE would call
 * back into a freed lua_State.
 */
void luavgl_detach_all(lua_State* state);

/** Applies a table of properties to `object`, the shared body of `set{}`. */
int luavgl_apply_properties(lua_State* state, lv_obj_t* object, int table_index);

/**
 * Creates a widget, wires it up and leaves its handle on the stack.
 *
 * `create` receives the resolved parent. Shared by every widget constructor so they all
 * handle the optional parent and the optional property table identically.
 */
using WidgetCreate = lv_obj_t* (*)(lv_obj_t* parent);
int luavgl_create_widget(lua_State* state, WidgetCreate create);
