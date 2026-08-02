#include "doctest.h"

#include <lua/bindings.h>
#include <lua/runtime.h>
#include <luavgl/bindings.h>
#include <lvgl.h>
#include <string>

extern "C" {
#include <lauxlib.h>
#include <lua.h>
}

/** Reusable styles and named fonts, checkpoint 6d. */

namespace {

lv_display_t* display = nullptr;
uint8_t* draw_buffer = nullptr;

void flush_nothing(lv_display_t* target, const lv_area_t*, uint8_t*) {
    lv_display_flush_ready(target);
}

struct LvglFixture {
    lv_obj_t* screen = nullptr;

    LvglFixture() {
        if (!lv_is_initialized()) {
            lv_init();
            display = lv_display_create(320, 240);
            REQUIRE(display != nullptr);
            const size_t size = 320 * 40 * 2;
            draw_buffer = new uint8_t[size];
            lv_display_set_buffers(display, draw_buffer, nullptr, size, LV_DISPLAY_RENDER_MODE_PARTIAL);
            lv_display_set_flush_cb(display, flush_nothing);
        }
        screen = lv_obj_create(nullptr);
        REQUIRE(screen != nullptr);
    }

    ~LvglFixture() {
        if (screen != nullptr) {
            lv_obj_delete(screen);
        }
    }
};

struct UiRuntime {
    LvglFixture lvgl;
    LuaRuntime* runtime = lua_runtime_alloc();

    UiRuntime() {
        lua_bindings_open(runtime);
        luavgl_bindings_open(runtime, lvgl.screen);
    }

    ~UiRuntime() {
        luavgl_bindings_close(runtime);
        lua_runtime_free(runtime);
    }

    error_t eval(const char* code) const { return lua_runtime_eval(runtime, code, "test"); }
    std::string error() const { return lua_runtime_get_error(runtime); }
};

}

TEST_CASE("a style applies its properties to a widget") {
    const UiRuntime lua;

    const auto* code =
        "local style = lvgl.Style{ bg_color = 0x112233, radius = 7 }\n"
        "local panel = lvgl.Object{ w = 100, h = 50 }\n"
        "panel:add_style(style)\n"
        "assert(panel:is_valid())\n";

    REQUIRE(lua.eval(code) == ERROR_NONE);
}

TEST_CASE("one style can be shared by several widgets") {
    const UiRuntime lua;

    const auto* code =
        "local style = lvgl.Style{ bg_color = 0x445566 }\n"
        "for i = 1, 5 do\n"
        "    lvgl.Object{ w = 10, h = 10 }:add_style(style)\n"
        "end\n";

    REQUIRE(lua.eval(code) == ERROR_NONE);
}

TEST_CASE("style properties can be changed after creation") {
    const UiRuntime lua;

    const auto* code =
        "local style = lvgl.Style()\n"
        "style:set{ bg_color = 0x000000 }\n"
        "style:set{ radius = 4, pad_all = 2 }\n";

    REQUIRE(lua.eval(code) == ERROR_NONE);
}

TEST_CASE("set returns the style so calls chain") {
    const UiRuntime lua;

    REQUIRE(lua.eval("local s = lvgl.Style():set{ radius = 2 }:set{ pad_all = 1 }\n") == ERROR_NONE);
}

TEST_CASE("an unknown style property raises rather than being ignored") {
    const UiRuntime lua;

    REQUIRE(lua.eval("lvgl.Style{ not_a_property = 1 }\n") != ERROR_NONE);
    CHECK(lua.error().find("not_a_property") != std::string::npos);
}

TEST_CASE("a style can be removed again") {
    const UiRuntime lua;

    const auto* code =
        "local style = lvgl.Style{ bg_color = 0x778899 }\n"
        "local panel = lvgl.Object{ w = 20, h = 20 }\n"
        "panel:add_style(style)\n"
        "panel:remove_style(style)\n"
        "assert(panel:is_valid())\n";

    REQUIRE(lua.eval(code) == ERROR_NONE);
}

TEST_CASE("add_style and remove_style return the widget") {
    const UiRuntime lua;

    const auto* code =
        "local style = lvgl.Style{ radius = 1 }\n"
        "local panel = lvgl.Object{ w = 10, h = 10 }\n"
        "assert(panel:add_style(style) == panel)\n"
        "assert(panel:remove_style(style) == panel)\n";

    REQUIRE(lua.eval(code) == ERROR_NONE);
}

TEST_CASE("a selector restricts a style to one part and state") {
    const UiRuntime lua;

    const auto* code =
        "local knob = lvgl.Style{ bg_color = 0xFF0000 }\n"
        "local slider = lvgl.Slider{ w = 100 }\n"
        "slider:add_style(knob, lvgl.PART.KNOB | lvgl.STATE.PRESSED)\n";

    REQUIRE(lua.eval(code) == ERROR_NONE);
}

TEST_CASE("part and state constants are exposed") {
    const UiRuntime lua;

    const auto* code =
        "for _, name in ipairs({'MAIN', 'INDICATOR', 'KNOB', 'ITEMS', 'ANY'}) do\n"
        "    assert(lvgl.PART[name], name)\n"
        "end\n"
        "for _, name in ipairs({'DEFAULT', 'CHECKED', 'PRESSED', 'DISABLED', 'ANY'}) do\n"
        "    assert(lvgl.STATE[name] ~= nil, name)\n"
        "end\n";

    REQUIRE(lua.eval(code) == ERROR_NONE);
}

TEST_CASE("passing a non-style to add_style raises") {
    const UiRuntime lua;

    REQUIRE(lua.eval("lvgl.Object{ w = 10, h = 10 }:add_style({})\n") != ERROR_NONE);
}

// Fonts -------------------------------------------------------------------------------------

TEST_CASE("every named font size is accepted as a widget property") {
    const UiRuntime lua;

    const auto* code =
        "for _, size in ipairs({'small', 'default', 'large', 'icon'}) do\n"
        "    lvgl.Label{ text = 'x', text_font = size }\n"
        "end\n";

    REQUIRE(lua.eval(code) == ERROR_NONE);
}

TEST_CASE("a font name is accepted on a style too") {
    const UiRuntime lua;

    REQUIRE(lua.eval("lvgl.Style{ text_font = 'large' }\n") == ERROR_NONE);
}

TEST_CASE("an unknown font name raises") {
    const UiRuntime lua;

    REQUIRE(lua.eval("lvgl.Label{ text_font = 'enormous' }\n") != ERROR_NONE);
    CHECK(lua.error().find("enormous") != std::string::npos);
}

// Lifetime ----------------------------------------------------------------------------------

/**
 * The hazard this whole design exists for.
 *
 * `lv_obj_add_style()` stores the style by pointer. A widget outliving the runtime that
 * styled it would dereference freed style memory on its next redraw, and unlike a widget
 * handle there is no LVGL event to catch it. The widget is deliberately parented outside
 * the runtime's root so closing the runtime does not take it with it.
 */
TEST_CASE("a widget outliving its runtime keeps no pointer to a freed style") {
    LvglFixture lvgl;

    auto* survivor = lv_obj_create(lvgl.screen);
    REQUIRE(survivor != nullptr);

    {
        LuaRuntime* runtime = lua_runtime_alloc();
        lua_bindings_open(runtime);
        luavgl_bindings_open(runtime, lvgl.screen);

        // Style the widget from Lua by finding it among the root's children
        const auto* code =
            "local style = lvgl.Style{ bg_color = 0x334455, radius = 5 }\n"
            "for _, child in ipairs(lvgl.root():children()) do\n"
            "    child:add_style(style)\n"
            "end\n";
        REQUIRE(lua_runtime_eval(runtime, code, "test") == ERROR_NONE);

        luavgl_bindings_close(runtime);
        lua_runtime_free(runtime);
    }

    // The styles are gone; touching the widget must not reach into freed memory. Under
    // AddressSanitizer a surviving pointer shows up here rather than at some later redraw.
    CHECK(lv_obj_is_valid(survivor));
    lv_obj_invalidate(survivor);
    lv_obj_set_width(survivor, 64);
    lv_obj_update_layout(survivor);
}

TEST_CASE("styles survive collection of their Lua handle") {
    const UiRuntime lua;

    // Nothing in Lua references the style after this, but the widget still does. Freeing it
    // on collection would leave the widget pointing at freed memory.
    const auto* code =
        "local panel = lvgl.Object{ w = 30, h = 30 }\n"
        "panel:add_style(lvgl.Style{ bg_color = 0x223344 })\n"
        "collectgarbage('collect')\n"
        "collectgarbage('collect')\n"
        "panel:update_layout()\n"
        "assert(panel:is_valid())\n";

    REQUIRE(lua.eval(code) == ERROR_NONE);
}

/**
 * The shape LuaSdkTest uses: a shared card style plus a caption style, applied through a
 * helper that returns the row. Guards the demo's structure against binding changes without
 * needing the kernel services the full script also calls.
 */
TEST_CASE("the shared-style row pattern from the demo app builds") {
    const UiRuntime lua;

    const auto* code =
        "local page = lvgl.Object{ w = lvgl.PCT(100), flex_grow = 1,\n"
        "                          flex_flow = lvgl.FLEX_FLOW.COLUMN }\n"
        "local card = lvgl.Style{ bg_color = 0x1c1c28, border_width = 0,\n"
        "                         radius = 6, pad_all = 6 }\n"
        "local caption = lvgl.Style{ text_color = 0x8899aa, text_font = 'small' }\n"
        "\n"
        "local function row(label, value)\n"
        "    local line = lvgl.Object(page, { w = lvgl.PCT(100), h = lvgl.SIZE_CONTENT,\n"
        "                                     flex_flow = lvgl.FLEX_FLOW.ROW })\n"
        "    line:add_style(card)\n"
        "    lvgl.Label(line, { text = label, w = 230, long_mode = lvgl.LONG.CLIP })\n"
        "        :add_style(caption)\n"
        "    lvgl.Label(line, { text = tostring(value), text_color = 0xffffff })\n"
        "    return line\n"
        "end\n"
        "\n"
        "for i = 1, 6 do row('Row ' .. i, i) end\n"
        "page:update_layout()\n"
        "assert(#page:children() == 6)\n";

    REQUIRE(lua.eval(code) == ERROR_NONE);
}

TEST_CASE("closing a runtime twice is safe") {
    LvglFixture lvgl;

    LuaRuntime* runtime = lua_runtime_alloc();
    lua_bindings_open(runtime);
    luavgl_bindings_open(runtime, lvgl.screen);

    REQUIRE(lua_runtime_eval(runtime, "lvgl.Object{ w = 5, h = 5 }:add_style(lvgl.Style{ radius = 1 })\n", "test") == ERROR_NONE);

    luavgl_bindings_close(runtime);
    luavgl_bindings_close(runtime);

    lua_runtime_free(runtime);
}
