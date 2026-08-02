// SPDX-License-Identifier: Apache-2.0
#include <luavgl/object_private.h>

#include <lvgl/widgets/spinner.h>

extern "C" {
#include <lauxlib.h>
}

/**
 * The core widget set beyond Object/Label/Button.
 *
 * Constructors follow `luavgl_create_widget`, so every one accepts an optional parent and
 * an optional property table in either order. Widget-specific properties live in
 * properties.cpp alongside the shared ones, dispatched on the widget's class - LVGL's
 * setters do not check, so writing a slider's value through a label would corrupt memory
 * rather than fail.
 *
 * Getters are methods rather than properties: `slider:value()` reads better than a
 * `get{}` counterpart to `set{}`, and it keeps the property table write-only, which is
 * what makes a single flat table workable.
 */

namespace {

// Constructors ----------------------------------------------------------------------------

int widget_slider(lua_State* state) {
    return luavgl_create_widget(state, [](lv_obj_t* parent) { return lv_slider_create(parent); });
}

int widget_switch(lua_State* state) {
    return luavgl_create_widget(state, [](lv_obj_t* parent) { return lv_switch_create(parent); });
}

int widget_checkbox(lua_State* state) {
    return luavgl_create_widget(state, [](lv_obj_t* parent) { return lv_checkbox_create(parent); });
}

int widget_bar(lua_State* state) {
    return luavgl_create_widget(state, [](lv_obj_t* parent) { return lv_bar_create(parent); });
}

int widget_dropdown(lua_State* state) {
    return luavgl_create_widget(state, [](lv_obj_t* parent) { return lv_dropdown_create(parent); });
}

int widget_roller(lua_State* state) {
    return luavgl_create_widget(state, [](lv_obj_t* parent) { return lv_roller_create(parent); });
}

int widget_textarea(lua_State* state) {
    return luavgl_create_widget(state, [](lv_obj_t* parent) { return lv_textarea_create(parent); });
}

int widget_line(lua_State* state) {
    return luavgl_create_widget(state, [](lv_obj_t* parent) { return lv_line_create(parent); });
}

int widget_image(lua_State* state) {
    return luavgl_create_widget(state, [](lv_obj_t* parent) { return lv_image_create(parent); });
}

/**
 * Tactility's spinner, not LVGL's.
 *
 * `LV_USE_SPINNER` is off in the ESP32 sdkconfig - widget availability is per-target, and
 * the simulator's lv_conf.h is not the firmware's - so lv_spinner_create() does not exist
 * there. `lvgl-module` provides a portable one and already exports it.
 */
int widget_spinner(lua_State* state) {
    return luavgl_create_widget(state, [](lv_obj_t* parent) { return lvgl_spinner_create(parent); });
}

// Value accessors -------------------------------------------------------------------------

/**
 * `widget:value()` - the current value of a slider, bar, dropdown or roller.
 *
 * One method rather than one per widget: a script reading "whatever this control is set
 * to" should not have to know which class it holds.
 */
int object_value(lua_State* state) {
    auto* object = luavgl_check_object(state, 1);

    if (lv_obj_check_type(object, &lv_slider_class)) {
        lua_pushinteger(state, lv_slider_get_value(object));
    } else if (lv_obj_check_type(object, &lv_bar_class)) {
        lua_pushinteger(state, lv_bar_get_value(object));
    } else if (lv_obj_check_type(object, &lv_dropdown_class)) {
        lua_pushinteger(state, lv_dropdown_get_selected(object) + 1); // 1-based, Lua style
    } else if (lv_obj_check_type(object, &lv_roller_class)) {
        lua_pushinteger(state, lv_roller_get_selected(object) + 1);
    } else {
        return luaL_error(state, "this widget has no value");
    }

    return 1;
}

/** `widget:text()` - the text of a label, checkbox, textarea, or a dropdown's selection. */
int object_text(lua_State* state) {
    auto* object = luavgl_check_object(state, 1);

    if (lv_obj_check_type(object, &lv_label_class)) {
        lua_pushstring(state, lv_label_get_text(object));
    } else if (lv_obj_check_type(object, &lv_checkbox_class)) {
        lua_pushstring(state, lv_checkbox_get_text(object));
    } else if (lv_obj_check_type(object, &lv_textarea_class)) {
        lua_pushstring(state, lv_textarea_get_text(object));
    } else if (lv_obj_check_type(object, &lv_dropdown_class)) {
        char buffer[128];
        lv_dropdown_get_selected_str(object, buffer, sizeof(buffer));
        lua_pushstring(state, buffer);
    } else if (lv_obj_check_type(object, &lv_roller_class)) {
        char buffer[128];
        lv_roller_get_selected_str(object, buffer, sizeof(buffer));
        lua_pushstring(state, buffer);
    } else {
        return luaL_error(state, "this widget has no text");
    }

    return 1;
}

/**
 * `widget:is_checked()` - state of a switch or checkbox.
 *
 * Both use LV_STATE_CHECKED rather than a value, so this reads the state flag.
 */
int object_is_checked(lua_State* state) {
    auto* object = luavgl_check_object(state, 1);

    if (!lv_obj_check_type(object, &lv_switch_class) &&
        !lv_obj_check_type(object, &lv_checkbox_class)) {
        return luaL_error(state, "this widget has no checked state");
    }

    lua_pushboolean(state, lv_obj_has_state(object, LV_STATE_CHECKED));
    return 1;
}

/** `widget:set_checked(bool)` - for switches and checkboxes. Returns the widget. */
int object_set_checked(lua_State* state) {
    auto* object = luavgl_check_object(state, 1);
    luaL_checktype(state, 2, LUA_TBOOLEAN);

    if (!lv_obj_check_type(object, &lv_switch_class) &&
        !lv_obj_check_type(object, &lv_checkbox_class)) {
        return luaL_error(state, "this widget has no checked state");
    }

    if (lua_toboolean(state, 2)) {
        lv_obj_add_state(object, LV_STATE_CHECKED);
    } else {
        lv_obj_remove_state(object, LV_STATE_CHECKED);
    }

    lua_settop(state, 1);
    return 1;
}

// Text area editing -----------------------------------------------------------------------

/** Raises unless the widget is a text area, for the editing methods below. */
lv_obj_t* check_textarea(lua_State* state) {
    auto* object = luavgl_check_object(state, 1);

    if (!lv_obj_check_type(object, &lv_textarea_class)) {
        luaL_error(state, "this widget is not a text area");
    }

    return object;
}

/**
 * `ta:cursor()` / `ta:set_cursor(position)` - the caret, as a 1-based character offset.
 *
 * 1-based to match the rest of Lua (and this binding's dropdown/roller selections), so a
 * script can index the text with the same number it gets back.
 */
int textarea_cursor(lua_State* state) {
    lua_pushinteger(state, lv_textarea_get_cursor_pos(check_textarea(state)) + 1);
    return 1;
}

int textarea_set_cursor(lua_State* state) {
    auto* object = check_textarea(state);
    const auto position = static_cast<int32_t>(luaL_checkinteger(state, 2));

    lv_textarea_set_cursor_pos(object, position - 1);

    lua_settop(state, 1);
    return 1;
}

/** `ta:insert(text)` - inserts at the cursor, as typing would. Returns the widget. */
int textarea_insert(lua_State* state) {
    auto* object = check_textarea(state);
    lv_textarea_add_text(object, luaL_checkstring(state, 2));

    lua_settop(state, 1);
    return 1;
}

/** `ta:delete_char()` - removes the character before the cursor. Returns the widget. */
int textarea_delete_char(lua_State* state) {
    auto* object = check_textarea(state);
    lv_textarea_delete_char(object);

    lua_settop(state, 1);
    return 1;
}

const luaL_Reg widget_methods[] = {
    { "value", object_value },
    { "text", object_text },
    { "is_checked", object_is_checked },
    { "set_checked", object_set_checked },
    { "cursor", textarea_cursor },
    { "set_cursor", textarea_set_cursor },
    { "insert", textarea_insert },
    { "delete_char", textarea_delete_char },
    { nullptr, nullptr }
};

const luaL_Reg widget_constructors[] = {
    { "Slider", widget_slider },
    { "Switch", widget_switch },
    { "Checkbox", widget_checkbox },
    { "Bar", widget_bar },
    { "Dropdown", widget_dropdown },
    { "Roller", widget_roller },
    { "TextArea", widget_textarea },
    { "Line", widget_line },
    { "Image", widget_image },
    { "Spinner", widget_spinner },
    { nullptr, nullptr }
};

} // namespace

void luavgl_add_widget_methods(lua_State* state) {
    luaL_setfuncs(state, widget_methods, 0);
}

void luavgl_open_widgets(lua_State* state) {
    luaL_setfuncs(state, widget_constructors, 0);
}
