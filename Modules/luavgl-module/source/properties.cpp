// SPDX-License-Identifier: Apache-2.0
#include <luavgl/object_private.h>

#include <cstring>

extern "C" {
#include <lauxlib.h>
}

/**
 * The declarative property setter behind `obj:set{ ... }`.
 *
 * One dispatcher over a name table rather than one binding per LVGL setter. This is the
 * single idea that keeps a widget binding to a few files instead of a few hundred: LVGL
 * has thousands of setters, and a script writes
 *
 *     label:set{ text = "hi", align = lvgl.ALIGN.CENTER, text_color = 0xFF0000 }
 *
 * rather than four separate calls.
 */

namespace {

/** Reads an integer, accepting LVGL's size constants unchanged. */
int32_t to_coordinate(lua_State* state, int index) {
    return static_cast<int32_t>(luaL_checkinteger(state, index));
}

/** Colours are given as 0xRRGGBB integers, which is how a script naturally writes them. */
lv_color_t to_color(lua_State* state, int index) {
    const auto value = static_cast<uint32_t>(luaL_checkinteger(state, index));
    return lv_color_hex(value);
}

using PropertySetter = void (*)(lua_State* state, lv_obj_t* object, int value_index);

struct Property {
    const char* name;
    PropertySetter set;
};

// Geometry ------------------------------------------------------------------------------

void set_width(lua_State* state, lv_obj_t* object, int index) {
    lv_obj_set_width(object, to_coordinate(state, index));
}

void set_height(lua_State* state, lv_obj_t* object, int index) {
    lv_obj_set_height(object, to_coordinate(state, index));
}

void set_x(lua_State* state, lv_obj_t* object, int index) {
    lv_obj_set_x(object, to_coordinate(state, index));
}

void set_y(lua_State* state, lv_obj_t* object, int index) {
    lv_obj_set_y(object, to_coordinate(state, index));
}

void set_align(lua_State* state, lv_obj_t* object, int index) {
    lv_obj_set_align(object, static_cast<lv_align_t>(luaL_checkinteger(state, index)));
}

// Text ----------------------------------------------------------------------------------

/**
 * Only meaningful on widgets with text. Checking the class keeps a typo like
 * `button:set{ text = ... }` from writing through the wrong widget's memory - LVGL's
 * setters do not validate this themselves.
 */
void set_text(lua_State* state, lv_obj_t* object, int index) {
    const char* text = luaL_checkstring(state, index);

    if (lv_obj_check_type(object, &lv_label_class)) {
        lv_label_set_text(object, text);
    } else if (lv_obj_check_type(object, &lv_textarea_class)) {
        lv_textarea_set_text(object, text);
    } else if (lv_obj_check_type(object, &lv_checkbox_class)) {
        lv_checkbox_set_text(object, text);
    } else {
        luaL_error(state, "this widget has no text");
    }
}

void set_long_mode(lua_State* state, lv_obj_t* object, int index) {
    if (!lv_obj_check_type(object, &lv_label_class)) {
        luaL_error(state, "long_mode only applies to labels");
    }
    lv_label_set_long_mode(object, static_cast<lv_label_long_mode_t>(luaL_checkinteger(state, index)));
}

// Value widgets ---------------------------------------------------------------------------

/** Slider and bar share a value model; dropdown and roller select by index. */
void set_value(lua_State* state, lv_obj_t* object, int index) {
    const auto value = static_cast<int32_t>(luaL_checkinteger(state, index));

    if (lv_obj_check_type(object, &lv_slider_class)) {
        lv_slider_set_value(object, value, LV_ANIM_OFF);
    } else if (lv_obj_check_type(object, &lv_bar_class)) {
        lv_bar_set_value(object, value, LV_ANIM_OFF);
    } else if (lv_obj_check_type(object, &lv_dropdown_class)) {
        // 1-based in Lua, 0-based in LVGL
        lv_dropdown_set_selected(object, static_cast<uint32_t>(value - 1));
    } else if (lv_obj_check_type(object, &lv_roller_class)) {
        lv_roller_set_selected(object, static_cast<uint32_t>(value - 1), LV_ANIM_OFF);
    } else {
        luaL_error(state, "this widget has no value");
    }
}

void set_min(lua_State* state, lv_obj_t* object, int index) {
    const auto value = static_cast<int32_t>(luaL_checkinteger(state, index));

    if (lv_obj_check_type(object, &lv_slider_class)) {
        lv_slider_set_min_value(object, value);
    } else if (lv_obj_check_type(object, &lv_bar_class)) {
        lv_bar_set_min_value(object, value);
    } else {
        luaL_error(state, "min only applies to sliders and bars");
    }
}

void set_max(lua_State* state, lv_obj_t* object, int index) {
    const auto value = static_cast<int32_t>(luaL_checkinteger(state, index));

    if (lv_obj_check_type(object, &lv_slider_class)) {
        lv_slider_set_max_value(object, value);
    } else if (lv_obj_check_type(object, &lv_bar_class)) {
        lv_bar_set_max_value(object, value);
    } else {
        luaL_error(state, "max only applies to sliders and bars");
    }
}

/**
 * `options` takes a Lua array, so a script writes { "One", "Two" } rather than assembling
 * LVGL's newline-separated string itself.
 */
void set_options(lua_State* state, lv_obj_t* object, int index) {
    const bool is_dropdown = lv_obj_check_type(object, &lv_dropdown_class);
    if (!is_dropdown && !lv_obj_check_type(object, &lv_roller_class)) {
        luaL_error(state, "options only applies to dropdowns and rollers");
    }

    luaL_checktype(state, index, LUA_TTABLE);

    luaL_Buffer buffer;
    luaL_buffinit(state, &buffer);

    const lua_Integer count = luaL_len(state, index);
    for (lua_Integer i = 1; i <= count; i++) {
        if (i > 1) {
            luaL_addchar(&buffer, '\n');
        }

        lua_geti(state, index, i);
        luaL_addstring(&buffer, luaL_tolstring(state, -1, nullptr));
        lua_pop(state, 2); // the tolstring result and the entry
    }

    luaL_pushresult(&buffer);

    if (is_dropdown) {
        lv_dropdown_set_options(object, lua_tostring(state, -1));
    } else {
        lv_roller_set_options(object, lua_tostring(state, -1), LV_ROLLER_MODE_NORMAL);
    }

    lua_pop(state, 1);
}

void set_checked(lua_State* state, lv_obj_t* object, int index) {
    luaL_checktype(state, index, LUA_TBOOLEAN);

    if (!lv_obj_check_type(object, &lv_switch_class) &&
        !lv_obj_check_type(object, &lv_checkbox_class)) {
        luaL_error(state, "checked only applies to switches and checkboxes");
    }

    if (lua_toboolean(state, index)) {
        lv_obj_add_state(object, LV_STATE_CHECKED);
    } else {
        lv_obj_remove_state(object, LV_STATE_CHECKED);
    }
}

/** Textarea-only conveniences. */
void set_placeholder(lua_State* state, lv_obj_t* object, int index) {
    if (!lv_obj_check_type(object, &lv_textarea_class)) {
        luaL_error(state, "placeholder only applies to text areas");
    }
    lv_textarea_set_placeholder_text(object, luaL_checkstring(state, index));
}

void set_one_line(lua_State* state, lv_obj_t* object, int index) {
    luaL_checktype(state, index, LUA_TBOOLEAN);

    if (!lv_obj_check_type(object, &lv_textarea_class)) {
        luaL_error(state, "one_line only applies to text areas");
    }
    lv_textarea_set_one_line(object, lua_toboolean(state, index) != 0);
}

void set_password_mode(lua_State* state, lv_obj_t* object, int index) {
    luaL_checktype(state, index, LUA_TBOOLEAN);

    if (!lv_obj_check_type(object, &lv_textarea_class)) {
        luaL_error(state, "password_mode only applies to text areas");
    }
    lv_textarea_set_password_mode(object, lua_toboolean(state, index) != 0);
}

/** An image source is a path the LVGL filesystem layer resolves, or a symbol string. */
void set_source(lua_State* state, lv_obj_t* object, int index) {
    if (!lv_obj_check_type(object, &lv_image_class)) {
        luaL_error(state, "src only applies to images");
    }
    lv_image_set_src(object, luaL_checkstring(state, index));
}

// Style -----------------------------------------------------------------------------------

void set_bg_color(lua_State* state, lv_obj_t* object, int index) {
    lv_obj_set_style_bg_color(object, to_color(state, index), LV_PART_MAIN);
}

void set_bg_opa(lua_State* state, lv_obj_t* object, int index) {
    lv_obj_set_style_bg_opa(object, static_cast<lv_opa_t>(luaL_checkinteger(state, index)), LV_PART_MAIN);
}

void set_text_color(lua_State* state, lv_obj_t* object, int index) {
    lv_obj_set_style_text_color(object, to_color(state, index), LV_PART_MAIN);
}

/** Named sizes rather than raw faces, so a script stays legible across display densities. */
void set_text_font(lua_State* state, lv_obj_t* object, int index) {
    lv_obj_set_style_text_font(object, luavgl_check_font(state, index), LV_PART_MAIN);
}

void set_border_color(lua_State* state, lv_obj_t* object, int index) {
    lv_obj_set_style_border_color(object, to_color(state, index), LV_PART_MAIN);
}

void set_border_width(lua_State* state, lv_obj_t* object, int index) {
    lv_obj_set_style_border_width(object, to_coordinate(state, index), LV_PART_MAIN);
}

void set_radius(lua_State* state, lv_obj_t* object, int index) {
    lv_obj_set_style_radius(object, to_coordinate(state, index), LV_PART_MAIN);
}

void set_pad_all(lua_State* state, lv_obj_t* object, int index) {
    lv_obj_set_style_pad_all(object, to_coordinate(state, index), LV_PART_MAIN);
}

/**
 * The gap between flex children.
 *
 * Distinct from pad_all, which is the space inside the container's edges: a column with
 * pad_all = 0 still separates its children by the theme's default row padding, which is
 * what makes a Lua-built screen look inset compared with a C++ one.
 */
void set_pad_row(lua_State* state, lv_obj_t* object, int index) {
    lv_obj_set_style_pad_row(object, to_coordinate(state, index), LV_PART_MAIN);
}

void set_pad_column(lua_State* state, lv_obj_t* object, int index) {
    lv_obj_set_style_pad_column(object, to_coordinate(state, index), LV_PART_MAIN);
}

void set_opa(lua_State* state, lv_obj_t* object, int index) {
    lv_obj_set_style_opa(object, static_cast<lv_opa_t>(luaL_checkinteger(state, index)), LV_PART_MAIN);
}

// Layout ----------------------------------------------------------------------------------

void set_flex_flow(lua_State* state, lv_obj_t* object, int index) {
    lv_obj_set_flex_flow(object, static_cast<lv_flex_flow_t>(luaL_checkinteger(state, index)));
}

void set_flex_grow(lua_State* state, lv_obj_t* object, int index) {
    lv_obj_set_flex_grow(object, static_cast<uint8_t>(luaL_checkinteger(state, index)));
}

/** Sorted by name so the lookup below can binary search. */
constexpr Property PROPERTIES[] = {
    { "align", set_align },
    { "bg_color", set_bg_color },
    { "bg_opa", set_bg_opa },
    { "border_color", set_border_color },
    { "border_width", set_border_width },
    { "checked", set_checked },
    { "flex_flow", set_flex_flow },
    { "flex_grow", set_flex_grow },
    { "h", set_height },
    { "height", set_height },
    { "long_mode", set_long_mode },
    { "max", set_max },
    { "min", set_min },
    { "one_line", set_one_line },
    { "opa", set_opa },
    { "options", set_options },
    { "pad_all", set_pad_all },
    { "pad_column", set_pad_column },
    { "pad_row", set_pad_row },
    { "password_mode", set_password_mode },
    { "placeholder", set_placeholder },
    { "radius", set_radius },
    { "src", set_source },
    { "text", set_text },
    { "text_color", set_text_color },
    { "text_font", set_text_font },
    { "value", set_value },
    { "w", set_width },
    { "width", set_width },
    { "x", set_x },
    { "y", set_y },
};

constexpr size_t PROPERTY_COUNT = sizeof(PROPERTIES) / sizeof(PROPERTIES[0]);

const Property* find_property(const char* name) {
    size_t low = 0;
    size_t high = PROPERTY_COUNT;

    while (low < high) {
        const size_t middle = (low + high) / 2;
        const int comparison = std::strcmp(name, PROPERTIES[middle].name);

        if (comparison == 0) {
            return &PROPERTIES[middle];
        }
        if (comparison < 0) {
            high = middle;
        } else {
            low = middle + 1;
        }
    }

    return nullptr;
}

} // namespace

int luavgl_apply_properties(lua_State* state, lv_obj_t* object, int table_index) {
    const int absolute_index = lua_absindex(state, table_index);

    lua_pushnil(state);
    while (lua_next(state, absolute_index) != 0) {
        // key at -2, value at -1

        if (lua_type(state, -2) != LUA_TSTRING) {
            lua_pop(state, 1);
            continue; // array parts of the table are not properties
        }

        const char* name = lua_tostring(state, -2);
        const Property* property = find_property(name);

        if (property == nullptr) {
            // Raising is deliberate: silently ignoring a typo like `colour = ...` would
            // leave the script looking correct while doing nothing.
            return luaL_error(state, "unknown property '%s'", name);
        }

        property->set(state, object, lua_absindex(state, -1));

        lua_pop(state, 1); // value; key stays for lua_next
    }

    return 0;
}
