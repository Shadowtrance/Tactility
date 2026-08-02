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

/** The core widget set, checkpoint 6c. */

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

TEST_CASE("every widget constructor exists and creates a valid widget") {
    const UiRuntime lua;

    const auto* code =
        "for _, name in ipairs({'Object', 'Label', 'Button', 'Slider', 'Switch',\n"
        "                       'Checkbox', 'Bar', 'Dropdown', 'Roller', 'TextArea',\n"
        "                       'Line', 'Image', 'Spinner'}) do\n"
        "  assert(type(lvgl[name]) == 'function', name .. ' is missing')\n"
        "  local w = lvgl[name]()\n"
        "  assert(w:is_valid(), name .. ' is not valid')\n"
        "end";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("widgets accept a parent and properties like the base ones") {
    const UiRuntime lua;

    const auto* code =
        "local panel = lvgl.Object()\n"
        "local slider = lvgl.Slider(panel, { w = 100, min = 0, max = 50, value = 25 })\n"
        "assert(slider:parent() == panel)\n"
        "assert(slider:value() == 25, 'value is ' .. slider:value())";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

// Value widgets ---------------------------------------------------------------------------

TEST_CASE("a slider holds a value within its range") {
    const UiRuntime lua;

    const auto* code =
        "local s = lvgl.Slider{ min = 0, max = 100, value = 42 }\n"
        "assert(s:value() == 42, 'value is ' .. s:value())\n"
        "s:set{ value = 80 }\n"
        "assert(s:value() == 80)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("a slider clamps to its range") {
    const UiRuntime lua;

    // LVGL clamps rather than erroring, which is the behaviour a script should see
    const auto* code =
        "local s = lvgl.Slider{ min = 10, max = 20, value = 100 }\n"
        "assert(s:value() == 20, 'value is ' .. s:value())";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("a bar holds a value") {
    const UiRuntime lua;

    const auto* code =
        "local b = lvgl.Bar{ min = 0, max = 10, value = 7 }\n"
        "assert(b:value() == 7, 'value is ' .. b:value())";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("min and max only apply to sliders and bars") {
    const UiRuntime lua;
    CHECK_NE(lua.eval("lvgl.Label{ min = 0 }"), ERROR_NONE);
    CHECK_NE(lua.eval("lvgl.Button{ max = 10 }"), ERROR_NONE);
}

TEST_CASE("value on a widget without one raises") {
    const UiRuntime lua;
    CHECK_NE(lua.eval("lvgl.Label():value()"), ERROR_NONE);
    CHECK_NE(lua.eval("lvgl.Label{ value = 5 }"), ERROR_NONE);
}

// Options ---------------------------------------------------------------------------------

TEST_CASE("a dropdown takes options as a lua array") {
    const UiRuntime lua;

    // A script should not have to build LVGL's newline-separated string itself
    const auto* code =
        "local d = lvgl.Dropdown{ options = { 'One', 'Two', 'Three' } }\n"
        "assert(d:value() == 1, 'selection is ' .. d:value())\n"
        "assert(d:text() == 'One', 'text is ' .. d:text())";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("dropdown selection is 1-based, matching lua") {
    const UiRuntime lua;

    const auto* code =
        "local d = lvgl.Dropdown{ options = { 'One', 'Two', 'Three' } }\n"
        "d:set{ value = 2 }\n"
        "assert(d:value() == 2, 'selection is ' .. d:value())\n"
        "assert(d:text() == 'Two', 'text is ' .. d:text())";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("a roller takes options and reports its selection") {
    const UiRuntime lua;

    const auto* code =
        "local r = lvgl.Roller{ options = { 'Red', 'Green', 'Blue' } }\n"
        "r:set{ value = 3 }\n"
        "assert(r:value() == 3, 'selection is ' .. r:value())\n"
        "assert(r:text() == 'Blue', 'text is ' .. r:text())";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("options accepts non-string entries by converting them") {
    const UiRuntime lua;

    const auto* code =
        "local d = lvgl.Dropdown{ options = { 1, 2, 3 } }\n"
        "assert(d:text() == '1', 'text is ' .. d:text())";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("options only applies to dropdowns and rollers") {
    const UiRuntime lua;
    CHECK_NE(lua.eval("lvgl.Label{ options = { 'a' } }"), ERROR_NONE);
    CHECK_NE(lua.eval("lvgl.Dropdown{ options = 'not a table' }"), ERROR_NONE);
}

// Checked state ---------------------------------------------------------------------------

TEST_CASE("a switch reports and sets its checked state") {
    const UiRuntime lua;

    const auto* code =
        "local s = lvgl.Switch()\n"
        "assert(s:is_checked() == false)\n"
        "s:set{ checked = true }\n"
        "assert(s:is_checked() == true)\n"
        "s:set_checked(false)\n"
        "assert(s:is_checked() == false)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("a checkbox has both text and checked state") {
    const UiRuntime lua;

    const auto* code =
        "local c = lvgl.Checkbox{ text = 'Enable', checked = true }\n"
        "assert(c:text() == 'Enable', 'text is ' .. c:text())\n"
        "assert(c:is_checked() == true)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("set_checked returns the widget so calls chain") {
    const UiRuntime lua;

    const auto* code =
        "local s = lvgl.Switch()\n"
        "assert(s:set_checked(true) == s)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("checked only applies to switches and checkboxes") {
    const UiRuntime lua;
    CHECK_NE(lua.eval("lvgl.Label{ checked = true }"), ERROR_NONE);
    CHECK_NE(lua.eval("lvgl.Label():is_checked()"), ERROR_NONE);
    CHECK_NE(lua.eval("lvgl.Label():set_checked(true)"), ERROR_NONE);
}

// Text area -------------------------------------------------------------------------------

TEST_CASE("a text area holds text and a placeholder") {
    const UiRuntime lua;

    const auto* code =
        "local t = lvgl.TextArea{ text = 'hello', placeholder = 'type here',\n"
        "                         one_line = true }\n"
        "assert(t:text() == 'hello', 'text is ' .. t:text())";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("password mode is settable") {
    const UiRuntime lua;

    const auto* code =
        "local t = lvgl.TextArea{ text = 'secret', password_mode = true }\n"
        "assert(t:is_valid())";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("text area properties do not apply to other widgets") {
    const UiRuntime lua;
    CHECK_NE(lua.eval("lvgl.Label{ placeholder = 'x' }"), ERROR_NONE);
    CHECK_NE(lua.eval("lvgl.Label{ one_line = true }"), ERROR_NONE);
    CHECK_NE(lua.eval("lvgl.Label{ password_mode = true }"), ERROR_NONE);
}

// Text accessor ---------------------------------------------------------------------------

TEST_CASE("text reads back from every widget that has it") {
    const UiRuntime lua;

    const auto* code =
        "assert(lvgl.Label{ text = 'label' }:text() == 'label')\n"
        "assert(lvgl.Checkbox{ text = 'check' }:text() == 'check')\n"
        "assert(lvgl.TextArea{ text = 'area' }:text() == 'area')";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("text on a widget without any raises") {
    const UiRuntime lua;
    CHECK_NE(lua.eval("lvgl.Slider():text()"), ERROR_NONE);
    CHECK(lua.error().find("text") != std::string::npos);
}

// Integration -----------------------------------------------------------------------------

TEST_CASE("widgets work with events") {
    const UiRuntime lua;

    CHECK_EQ(lua.eval(
        "slider = lvgl.Slider{ min = 0, max = 100, value = 0 }\n"
        "seen = -1\n"
        "slider:on(lvgl.EVENT.VALUE_CHANGED, function(w) seen = w:value() end)\n"
        "slider:set{ value = 60 }\n"), ERROR_NONE);

    auto* state = static_cast<lua_State*>(lua_runtime_get_state(lua.runtime));
    lua_getglobal(state, "slider");
    auto* handle = static_cast<lv_obj_t**>(lua_touserdata(state, -1));
    REQUIRE(handle != nullptr);
    lv_obj_t* slider = *handle;
    lua_pop(state, 1);

    lv_obj_send_event(slider, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_timer_handler(); // handlers are queued, not inline

    CHECK_EQ(lua.eval("assert(seen == 60, 'seen is ' .. seen)"), ERROR_NONE);
}

TEST_CASE("a deleted widget of any type raises on use") {
    const UiRuntime lua;

    const auto* code =
        "for _, name in ipairs({'Slider', 'Switch', 'Checkbox', 'Dropdown', 'TextArea'}) do\n"
        "  local w = lvgl[name]()\n"
        "  w:delete()\n"
        "  local ok = pcall(function() w:set{ w = 10 } end)\n"
        "  assert(not ok, name .. ' did not raise after delete')\n"
        "end";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("many widgets can be created and torn down without leaking") {
    const UiRuntime lua;

    CHECK_EQ(lua.eval("collectgarbage('collect')"), ERROR_NONE);
    const size_t baseline = lua_runtime_get_memory_used(lua.runtime);

    const auto* code =
        "local panel = lvgl.Object()\n"
        "for i = 1, 50 do\n"
        "  lvgl.Slider(panel, { min = 0, max = 100, value = i })\n"
        "  lvgl.Checkbox(panel, { text = 'row ' .. i })\n"
        "end\n"
        "panel:delete()\n"
        "collectgarbage('collect')\n"
        "collectgarbage('collect')";
    CHECK_EQ(lua.eval(code), ERROR_NONE);

    CHECK(lua_runtime_get_memory_used(lua.runtime) < baseline + 65536);
}

// Text area editing -------------------------------------------------------------------------

TEST_CASE("a text area reports and moves its cursor, 1-based") {
    const UiRuntime lua;

    const auto* code =
        "local ta = lvgl.TextArea{ text = 'hello' }\n"
        "-- the caret sits after the inserted text\n"
        "assert(ta:cursor() == 6, 'got ' .. ta:cursor())\n"
        "ta:set_cursor(1)\n"
        "assert(ta:cursor() == 1, 'got ' .. ta:cursor())\n"
        "ta:set_cursor(3)\n"
        "assert(ta:cursor() == 3, 'got ' .. ta:cursor())\n";

    REQUIRE(lua.eval(code) == ERROR_NONE);
}

TEST_CASE("a text area inserts at the cursor") {
    const UiRuntime lua;

    // Cursor n means "before character n", so 4 in 'held' inserts between 'hel' and 'd'.
    const auto* code =
        "local ta = lvgl.TextArea{ text = 'held' }\n"
        "ta:set_cursor(4)\n"
        "ta:insert('l')\n"
        "assert(ta:text() == 'helld', 'got ' .. ta:text())\n"
        "-- the caret follows the inserted text\n"
        "assert(ta:cursor() == 5, 'got ' .. ta:cursor())\n"
        "\n"
        "-- inserting at 1 prepends\n"
        "local other = lvgl.TextArea{ text = 'ello' }\n"
        "other:set_cursor(1)\n"
        "other:insert('h')\n"
        "assert(other:text() == 'hello', 'got ' .. other:text())\n";

    REQUIRE(lua.eval(code) == ERROR_NONE);
}

TEST_CASE("a text area deletes the character before the cursor") {
    const UiRuntime lua;

    const auto* code =
        "local ta = lvgl.TextArea{ text = 'hello' }\n"
        "ta:delete_char()\n"
        "assert(ta:text() == 'hell', 'got ' .. ta:text())\n";

    REQUIRE(lua.eval(code) == ERROR_NONE);
}

TEST_CASE("editing methods chain and return the widget") {
    const UiRuntime lua;

    const auto* code =
        "local ta = lvgl.TextArea{ text = '' }\n"
        "assert(ta:insert('ab'):delete_char():insert('c') == ta)\n"
        "assert(ta:text() == 'ac', 'got ' .. ta:text())\n";

    REQUIRE(lua.eval(code) == ERROR_NONE);
}

TEST_CASE("editing methods raise on a widget that is not a text area") {
    const UiRuntime lua;

    CHECK(lua.eval("lvgl.Label{ text = 'x' }:insert('y')\n") != ERROR_NONE);
    CHECK(lua.eval("lvgl.Label{ text = 'x' }:cursor()\n") != ERROR_NONE);
    CHECK(lua.eval("lvgl.Slider{}:set_cursor(1)\n") != ERROR_NONE);
}

TEST_CASE("key constants are exposed") {
    const UiRuntime lua;

    const auto* code =
        "for _, name in ipairs({'UP', 'DOWN', 'LEFT', 'RIGHT', 'ESC', 'DEL',\n"
        "                       'BACKSPACE', 'ENTER', 'NEXT', 'PREV', 'HOME', 'END'}) do\n"
        "    assert(lvgl.KEY[name], name)\n"
        "end\n"
        "-- Printable characters are ASCII, not in this table\n"
        "assert(lvgl.KEY.ENTER ~= string.byte('a'))\n";

    REQUIRE(lua.eval(code) == ERROR_NONE);
}
