// SPDX-License-Identifier: Apache-2.0
#include <luavgl/object_private.h>

#include <lvgl/fonts.h>

#include <cstring>

extern "C" {
#include <lauxlib.h>
}

/**
 * Reusable styles: `lvgl.Style{ bg_color = 0x202020, radius = 8 }`.
 *
 * A style is worth having over repeating a `set{}` table because LVGL applies it by
 * reference - twenty rows sharing one style cost one style, and restyling them all is one
 * edit rather than twenty.
 *
 * That reference is also the hazard. `lv_obj_add_style()` stores the `lv_style_t*`; it does
 * not copy. A style collected by Lua while a widget still points at it is a use-after-free
 * on the next redraw, and unlike a widget handle there is no LVGL event to warn us. So:
 *
 *  - Every style is pinned in the registry for the life of the runtime. Collecting the Lua
 *    handle does not free the underlying style.
 *  - `luavgl_detach_all()` removes each style from every widget still using it before
 *    freeing, so a widget outliving the runtime cannot dereference a dead style.
 */

namespace {

constexpr auto* STYLE_METATABLE = "lvgl.Style";

/**
 * "Every part, in every state" - the default for removal.
 *
 * Built by casting each half before the OR: LVGL declares parts and states as separate
 * enums, and C++ deprecates a bitwise operation between two different enumeration types.
 */
constexpr lv_style_selector_t ANY_SELECTOR =
    static_cast<lv_style_selector_t>(LV_PART_ANY) | static_cast<lv_style_selector_t>(LV_STATE_ANY);

/** Registry key for the list of live styles, so shutdown can clean them all up. */
char STYLES_KEY = 0;

struct StyleHandle {
    lv_style_t style;
    /** The root the style was applied under, so detach knows where to look. */
    lua_State* state;
    bool initialised;
};

void push_styles(lua_State* state) {
    lua_pushlightuserdata(state, &STYLES_KEY);
    if (lua_rawget(state, LUA_REGISTRYINDEX) != LUA_TTABLE) {
        lua_pop(state, 1);
        lua_newtable(state);

        lua_pushlightuserdata(state, &STYLES_KEY);
        lua_pushvalue(state, -2);
        lua_rawset(state, LUA_REGISTRYINDEX);
    }
}

StyleHandle* check_style(lua_State* state, int index) {
    return static_cast<StyleHandle*>(luaL_checkudata(state, index, STYLE_METATABLE));
}

// Property setters --------------------------------------------------------------------------

using StyleSetter = void (*)(lua_State* state, lv_style_t* style, int value_index);

struct StyleProperty {
    const char* name;
    StyleSetter set;
};

int32_t to_coordinate(lua_State* state, int index) {
    return static_cast<int32_t>(luaL_checkinteger(state, index));
}

lv_color_t to_color(lua_State* state, int index) {
    return lv_color_hex(static_cast<uint32_t>(luaL_checkinteger(state, index)));
}

void style_bg_color(lua_State* s, lv_style_t* t, int i) { lv_style_set_bg_color(t, to_color(s, i)); }
void style_bg_opa(lua_State* s, lv_style_t* t, int i) { lv_style_set_bg_opa(t, static_cast<lv_opa_t>(luaL_checkinteger(s, i))); }
void style_text_color(lua_State* s, lv_style_t* t, int i) { lv_style_set_text_color(t, to_color(s, i)); }
void style_border_color(lua_State* s, lv_style_t* t, int i) { lv_style_set_border_color(t, to_color(s, i)); }
void style_border_width(lua_State* s, lv_style_t* t, int i) { lv_style_set_border_width(t, to_coordinate(s, i)); }
void style_radius(lua_State* s, lv_style_t* t, int i) { lv_style_set_radius(t, to_coordinate(s, i)); }
void style_pad_all(lua_State* s, lv_style_t* t, int i) { lv_style_set_pad_all(t, to_coordinate(s, i)); }
void style_pad_row(lua_State* s, lv_style_t* t, int i) { lv_style_set_pad_row(t, to_coordinate(s, i)); }
void style_pad_column(lua_State* s, lv_style_t* t, int i) { lv_style_set_pad_column(t, to_coordinate(s, i)); }
void style_opa(lua_State* s, lv_style_t* t, int i) { lv_style_set_opa(t, static_cast<lv_opa_t>(luaL_checkinteger(s, i))); }
void style_width(lua_State* s, lv_style_t* t, int i) { lv_style_set_width(t, to_coordinate(s, i)); }
void style_height(lua_State* s, lv_style_t* t, int i) { lv_style_set_height(t, to_coordinate(s, i)); }

/**
 * Fonts are named by semantic size rather than pointer.
 *
 * `lvgl-module` picks the actual face per display density, so a script asking for "large"
 * gets something legible on a 320x240 panel and on a 1280x720 one. Exposing raw
 * `lv_font_t*` would tie scripts to one device.
 */
void style_text_font(lua_State* state, lv_style_t* style, int index) {
    lv_style_set_text_font(style, luavgl_check_font(state, index));
}

/** Sorted for the binary search below. */
constexpr StyleProperty STYLE_PROPERTIES[] = {
    { "bg_color", style_bg_color },
    { "bg_opa", style_bg_opa },
    { "border_color", style_border_color },
    { "border_width", style_border_width },
    { "h", style_height },
    { "height", style_height },
    { "opa", style_opa },
    { "pad_all", style_pad_all },
    { "pad_column", style_pad_column },
    { "pad_row", style_pad_row },
    { "radius", style_radius },
    { "text_color", style_text_color },
    { "text_font", style_text_font },
    { "w", style_width },
    { "width", style_width },
};

constexpr size_t STYLE_PROPERTY_COUNT = sizeof(STYLE_PROPERTIES) / sizeof(STYLE_PROPERTIES[0]);

const StyleProperty* find_style_property(const char* name) {
    size_t low = 0;
    size_t high = STYLE_PROPERTY_COUNT;

    while (low < high) {
        const size_t middle = (low + high) / 2;
        const int comparison = std::strcmp(name, STYLE_PROPERTIES[middle].name);

        if (comparison == 0) {
            return &STYLE_PROPERTIES[middle];
        }
        if (comparison < 0) {
            high = middle;
        } else {
            low = middle + 1;
        }
    }

    return nullptr;
}

/** Applies a table of properties to a style. Shared by the constructor and `set{}`. */
void apply_style_properties(lua_State* state, lv_style_t* style, int table_index) {
    const int absolute = lua_absindex(state, table_index);

    lua_pushnil(state);
    while (lua_next(state, absolute) != 0) {
        if (lua_type(state, -2) != LUA_TSTRING) {
            lua_pop(state, 1);
            continue;
        }

        const char* name = lua_tostring(state, -2);
        const StyleProperty* property = find_style_property(name);

        if (property == nullptr) {
            // Raising rather than ignoring, exactly as obj:set{} does: a silently dropped
            // typo leaves a script looking correct while doing nothing.
            luaL_error(state, "unknown style property '%s'", name);
            return;
        }

        property->set(state, style, lua_absindex(state, -1));
        lua_pop(state, 1);
    }
}

// Methods -------------------------------------------------------------------------------

int style_set(lua_State* state) {
    auto* handle = check_style(state, 1);
    luaL_checktype(state, 2, LUA_TTABLE);

    apply_style_properties(state, &handle->style, 2);

    lua_settop(state, 1);
    return 1;
}

int style_tostring(lua_State* state) {
    lua_pushfstring(state, "lvgl.Style (%p)", static_cast<void*>(check_style(state, 1)));
    return 1;
}

const luaL_Reg style_methods[] = {
    { "set", style_set },
    { nullptr, nullptr }
};

// Object-side methods ---------------------------------------------------------------------

/**
 * `widget:add_style(style [, selector])` - applies a style. Returns the widget.
 *
 * The selector is LVGL's part/state mask; the default of 0 means "the main part, in any
 * state", which is what a script writing `panel:add_style(card)` means.
 */
int object_add_style(lua_State* state) {
    auto* object = luavgl_check_object(state, 1);
    auto* handle = check_style(state, 2);
    const auto selector = static_cast<lv_style_selector_t>(luaL_optinteger(state, 3, 0));

    lv_obj_add_style(object, &handle->style, selector);

    lua_settop(state, 1);
    return 1;
}

/** `widget:remove_style(style [, selector])` - removes it again. Returns the widget. */
int object_remove_style(lua_State* state) {
    auto* object = luavgl_check_object(state, 1);
    auto* handle = check_style(state, 2);
    const auto selector = static_cast<lv_style_selector_t>(
        luaL_optinteger(state, 3, ANY_SELECTOR));

    lv_obj_remove_style(object, &handle->style, selector);

    lua_settop(state, 1);
    return 1;
}

const luaL_Reg object_style_methods[] = {
    { "add_style", object_add_style },
    { "remove_style", object_remove_style },
    { nullptr, nullptr }
};

int style_create(lua_State* state) {
    const bool has_properties = lua_istable(state, 1);

    auto* handle = static_cast<StyleHandle*>(lua_newuserdatauv(state, sizeof(StyleHandle), 0));
    handle->state = state;
    handle->initialised = false;

    lv_style_init(&handle->style);
    handle->initialised = true;

    luaL_setmetatable(state, STYLE_METATABLE);

    if (has_properties) {
        apply_style_properties(state, &handle->style, 1);
    }

    // Pinned for the life of the runtime: LVGL holds the style by pointer, so collecting
    // the Lua handle while a widget still references it would be a use-after-free.
    push_styles(state);
    lua_pushvalue(state, -2);
    lua_rawseti(state, -2, luaL_len(state, -2) + 1);
    lua_pop(state, 1);

    return 1;
}

} // namespace

const lv_font_t* luavgl_check_font(lua_State* state, int index) {
    const char* name = luaL_checkstring(state, index);

    if (std::strcmp(name, "small") == 0) {
        return lvgl_get_text_font(FONT_SIZE_SMALL);
    }
    if (std::strcmp(name, "default") == 0) {
        return lvgl_get_text_font(FONT_SIZE_DEFAULT);
    }
    if (std::strcmp(name, "large") == 0) {
        return lvgl_get_text_font(FONT_SIZE_LARGE);
    }
    if (std::strcmp(name, "icon") == 0) {
        return lvgl_get_shared_icon_font();
    }

    luaL_error(state, "unknown font '%s' - use 'small', 'default', 'large' or 'icon'", name);
    return nullptr; // unreachable: luaL_error does not return
}

void luavgl_open_style(lua_State* state) {
    luaL_newmetatable(state, STYLE_METATABLE);

    luaL_newlib(state, style_methods);
    lua_setfield(state, -2, "__index");

    lua_pushcfunction(state, style_tostring);
    lua_setfield(state, -2, "__tostring");

    lua_pushboolean(state, 0);
    lua_setfield(state, -2, "__metatable");

    lua_pop(state, 1);

    lua_pushcfunction(state, style_create);
    lua_setfield(state, -2, "Style");
}

void luavgl_add_style_methods(lua_State* state) {
    luaL_setfuncs(state, object_style_methods, 0);
}

lv_style_t* luavgl_check_style(lua_State* state, int index) {
    return &check_style(state, index)->style;
}

void luavgl_remove_styles(lua_State* state, lv_obj_t* object) {
    push_styles(state);

    // Each style is removed by pointer rather than the whole widget being cleared with
    // LV_STYLE_PROP_ANY: a widget may also carry styles a C++ app applied, and those are
    // not this runtime's to strip.
    const lua_Integer count = luaL_len(state, -1);
    for (lua_Integer i = 1; i <= count; i++) {
        lua_rawgeti(state, -1, i);

        if (auto* handle = static_cast<StyleHandle*>(lua_touserdata(state, -1))) {
            if (handle->initialised) {
                lv_obj_remove_style(object, &handle->style, ANY_SELECTOR);
            }
        }

        lua_pop(state, 1);
    }

    lua_pop(state, 1);
}

void luavgl_free_all_styles(lua_State* state) {
    push_styles(state);

    const lua_Integer count = luaL_len(state, -1);
    for (lua_Integer i = 1; i <= count; i++) {
        lua_rawgeti(state, -1, i);

        if (auto* handle = static_cast<StyleHandle*>(lua_touserdata(state, -1))) {
            if (handle->initialised) {
                // Frees any property memory the style allocated. Widgets referencing it
                // have already had it removed by luavgl_detach_all().
                lv_style_reset(&handle->style);
                handle->initialised = false;
            }
        }

        lua_pop(state, 1);
    }

    lua_pop(state, 1);

    lua_pushlightuserdata(state, &STYLES_KEY);
    lua_pushnil(state);
    lua_rawset(state, LUA_REGISTRYINDEX);
}
