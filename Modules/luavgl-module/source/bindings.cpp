// SPDX-License-Identifier: Apache-2.0
#include <luavgl/bindings.h>
#include <luavgl/object_private.h>

#include <lua/runtime.h>

extern "C" {
#include <lauxlib.h>
}

namespace {

// Widgets ---------------------------------------------------------------------------------

int widget_object(lua_State* state) {
    return luavgl_create_widget(state, [](lv_obj_t* parent) { return lv_obj_create(parent); });
}

int widget_label(lua_State* state) {
    return luavgl_create_widget(state, [](lv_obj_t* parent) { return lv_label_create(parent); });
}

int widget_button(lua_State* state) {
    return luavgl_create_widget(state, [](lv_obj_t* parent) { return lv_button_create(parent); });
}

// Screen ----------------------------------------------------------------------------------

/** The object new widgets parent to by default. */
int lvgl_root(lua_State* state) {
    luavgl_push_object(state, luavgl_get_root(state));
    return 1;
}

int lvgl_horizontal_resolution(lua_State* state) {
    lua_pushinteger(state, lv_display_get_horizontal_resolution(lv_display_get_default()));
    return 1;
}

int lvgl_vertical_resolution(lua_State* state) {
    lua_pushinteger(state, lv_display_get_vertical_resolution(lv_display_get_default()));
    return 1;
}

/** LV_PCT is a macro taking an argument, so it cannot be a plain constant. */
int lvgl_percent(lua_State* state) {
    lua_pushinteger(state, LV_PCT(luaL_checkinteger(state, 1)));
    return 1;
}

const luaL_Reg lvgl_functions[] = {
    { "Object", widget_object },
    { "Label", widget_label },
    { "Button", widget_button },
    { "root", lvgl_root },
    { "HOR_RES", lvgl_horizontal_resolution },
    { "VER_RES", lvgl_vertical_resolution },
    { "PCT", lvgl_percent },
    { nullptr, nullptr }
};

// Constants -------------------------------------------------------------------------------

struct Constant {
    const char* name;
    lua_Integer value;
};

/** Adds a named sub-table of integer constants to the table on top of the stack. */
void add_constants(lua_State* state, const char* name, const Constant* constants, size_t count) {
    lua_createtable(state, 0, static_cast<int>(count));

    for (size_t i = 0; i < count; i++) {
        lua_pushinteger(state, constants[i].value);
        lua_setfield(state, -2, constants[i].name);
    }

    lua_setfield(state, -2, name);
}

const Constant ALIGN_CONSTANTS[] = {
    { "DEFAULT", LV_ALIGN_DEFAULT },
    { "TOP_LEFT", LV_ALIGN_TOP_LEFT },
    { "TOP_MID", LV_ALIGN_TOP_MID },
    { "TOP_RIGHT", LV_ALIGN_TOP_RIGHT },
    { "BOTTOM_LEFT", LV_ALIGN_BOTTOM_LEFT },
    { "BOTTOM_MID", LV_ALIGN_BOTTOM_MID },
    { "BOTTOM_RIGHT", LV_ALIGN_BOTTOM_RIGHT },
    { "LEFT_MID", LV_ALIGN_LEFT_MID },
    { "RIGHT_MID", LV_ALIGN_RIGHT_MID },
    { "CENTER", LV_ALIGN_CENTER },
};

const Constant FLEX_FLOW_CONSTANTS[] = {
    { "ROW", LV_FLEX_FLOW_ROW },
    { "COLUMN", LV_FLEX_FLOW_COLUMN },
    { "ROW_WRAP", LV_FLEX_FLOW_ROW_WRAP },
    { "COLUMN_WRAP", LV_FLEX_FLOW_COLUMN_WRAP },
};

const Constant LONG_MODE_CONSTANTS[] = {
    { "WRAP", LV_LABEL_LONG_MODE_WRAP },
    { "DOTS", LV_LABEL_LONG_MODE_DOTS },
    { "SCROLL", LV_LABEL_LONG_MODE_SCROLL },
    { "SCROLL_CIRCULAR", LV_LABEL_LONG_MODE_SCROLL_CIRCULAR },
    { "CLIP", LV_LABEL_LONG_MODE_CLIP },
};

const Constant FLAG_CONSTANTS[] = {
    { "HIDDEN", LV_OBJ_FLAG_HIDDEN },
    { "CLICKABLE", LV_OBJ_FLAG_CLICKABLE },
    { "SCROLLABLE", LV_OBJ_FLAG_SCROLLABLE },
};

const Constant OPACITY_CONSTANTS[] = {
    { "TRANSP", LV_OPA_TRANSP },
    { "COVER", LV_OPA_COVER },
    { "50", LV_OPA_50 },
};

/**
 * Selectors for `widget:add_style(style, selector)`.
 *
 * A selector is a part OR'd with a state - `lvgl.PART.KNOB | lvgl.STATE.PRESSED` styles a
 * slider's knob only while it is held. Without these a script can only style the main part
 * in its default state, which leaves most of LVGL's styling unreachable.
 */
const Constant PART_CONSTANTS[] = {
    { "MAIN", LV_PART_MAIN },
    { "SCROLLBAR", LV_PART_SCROLLBAR },
    { "INDICATOR", LV_PART_INDICATOR },
    { "KNOB", LV_PART_KNOB },
    { "SELECTED", LV_PART_SELECTED },
    { "ITEMS", LV_PART_ITEMS },
    { "CURSOR", LV_PART_CURSOR },
    { "ANY", LV_PART_ANY },
};

const Constant STATE_CONSTANTS[] = {
    { "DEFAULT", LV_STATE_DEFAULT },
    { "CHECKED", LV_STATE_CHECKED },
    { "FOCUSED", LV_STATE_FOCUSED },
    { "EDITED", LV_STATE_EDITED },
    { "HOVERED", LV_STATE_HOVERED },
    { "PRESSED", LV_STATE_PRESSED },
    { "SCROLLED", LV_STATE_SCROLLED },
    { "DISABLED", LV_STATE_DISABLED },
    { "ANY", LV_STATE_ANY },
};

} // namespace

extern "C" error_t luavgl_bindings_open(struct LuaRuntime* runtime, lv_obj_t* root) {
    auto* state = static_cast<lua_State*>(lua_runtime_get_state(runtime));

    luavgl_open_object(state);
    luavgl_set_root(state, root);

    luaL_newlib(state, lvgl_functions);
    luavgl_open_widgets(state); // the rest of the widget constructors

    add_constants(state, "ALIGN", ALIGN_CONSTANTS, sizeof(ALIGN_CONSTANTS) / sizeof(Constant));
    add_constants(state, "FLEX_FLOW", FLEX_FLOW_CONSTANTS, sizeof(FLEX_FLOW_CONSTANTS) / sizeof(Constant));
    add_constants(state, "LONG", LONG_MODE_CONSTANTS, sizeof(LONG_MODE_CONSTANTS) / sizeof(Constant));
    add_constants(state, "FLAG", FLAG_CONSTANTS, sizeof(FLAG_CONSTANTS) / sizeof(Constant));
    add_constants(state, "OPA", OPACITY_CONSTANTS, sizeof(OPACITY_CONSTANTS) / sizeof(Constant));
    add_constants(state, "PART", PART_CONSTANTS, sizeof(PART_CONSTANTS) / sizeof(Constant));
    add_constants(state, "STATE", STATE_CONSTANTS, sizeof(STATE_CONSTANTS) / sizeof(Constant));

    // Sizing helpers, which are macros in C and so need explicit exposure
    lua_pushinteger(state, LV_SIZE_CONTENT);
    lua_setfield(state, -2, "SIZE_CONTENT");

    // LVGL's built-in icons. These are UTF-8 strings in the symbol font, not image paths,
    // so a script needs them by name - passing an arbitrary string where an icon is
    // expected renders nothing at all.
    lua_newtable(state);
    const struct { const char* name; const char* symbol; } symbols[] = {
        { "CLOSE", LV_SYMBOL_CLOSE },
        { "OK", LV_SYMBOL_OK },
        { "LEFT", LV_SYMBOL_LEFT },
        { "RIGHT", LV_SYMBOL_RIGHT },
        { "SETTINGS", LV_SYMBOL_SETTINGS },
        { "REFRESH", LV_SYMBOL_REFRESH },
        { "SAVE", LV_SYMBOL_SAVE },
        { "TRASH", LV_SYMBOL_TRASH },
        { "HOME", LV_SYMBOL_HOME },
        { "WIFI", LV_SYMBOL_WIFI },
        { "BATTERY_FULL", LV_SYMBOL_BATTERY_FULL },
    };
    for (const auto& entry : symbols) {
        lua_pushstring(state, entry.symbol);
        lua_setfield(state, -2, entry.name);
    }
    lua_setfield(state, -2, "SYMBOL");

    luavgl_open_toolbar(state);
    luavgl_open_events(state);
    luavgl_open_timer(state);
    luavgl_open_style(state);

    lua_setglobal(state, "lvgl");

    return ERROR_NONE;
}

extern "C" void luavgl_bindings_close(struct LuaRuntime* runtime) {
    luavgl_detach_all(static_cast<lua_State*>(lua_runtime_get_state(runtime)));
}
