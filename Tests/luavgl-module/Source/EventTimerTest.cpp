#include "doctest.h"

#include <lua/bindings.h>
#include <lua/runtime.h>
#include <luavgl/bindings.h>
#include <luavgl/object_private.h>
#include <lvgl.h>
#include <string>

extern "C" {
#include <lauxlib.h>
#include <lua.h>
}

/**
 * Events and timers, checkpoint 6b.
 *
 * Handlers are dispatched through lv_async_call, and timers fire from lv_timer_handler(),
 * so every test expecting a callback has to pump LVGL rather than assume the call happened
 * inline.
 */

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

    /** The lv_obj_t behind a global holding a widget handle. */
    lv_obj_t* widget(const char* global) const {
        auto* state = static_cast<lua_State*>(lua_runtime_get_state(runtime));
        lua_getglobal(state, global);
        auto* handle = static_cast<lv_obj_t**>(lua_touserdata(state, -1));
        lua_pop(state, 1);
        return handle != nullptr ? *handle : nullptr;
    }
};

/** Runs LVGL long enough for async calls and timers to fire. */
void pump(uint32_t milliseconds = 0) {
    if (milliseconds > 0) {
        lv_tick_inc(milliseconds);
    }
    lv_timer_handler();
}

}

// Events ------------------------------------------------------------------------------------

TEST_CASE("the EVENT constants are present") {
    const UiRuntime lua;

    const auto* code =
        "assert(type(lvgl.EVENT) == 'table')\n"
        "for _, name in ipairs({'CLICKED', 'PRESSED', 'VALUE_CHANGED', 'LONG_PRESSED'}) do\n"
        "  assert(type(lvgl.EVENT[name]) == 'number', name)\n"
        "end";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("a handler runs when the event fires") {
    const UiRuntime lua;

    CHECK_EQ(lua.eval(
        "button = lvgl.Button()\n"
        "fired = 0\n"
        "button:on(lvgl.EVENT.CLICKED, function() fired = fired + 1 end)\n"), ERROR_NONE);

    lv_obj_send_event(lua.widget("button"), LV_EVENT_CLICKED, nullptr);
    pump();

    CHECK_EQ(lua.eval("assert(fired == 1, 'fired ' .. fired)"), ERROR_NONE);
}

TEST_CASE("a handler fires every time, unlike a toolbar action") {
    const UiRuntime lua;

    CHECK_EQ(lua.eval(
        "button = lvgl.Button()\n"
        "fired = 0\n"
        "button:on(lvgl.EVENT.CLICKED, function() fired = fired + 1 end)\n"), ERROR_NONE);

    lv_obj_t* button = lua.widget("button");
    for (int i = 0; i < 3; i++) {
        lv_obj_send_event(button, LV_EVENT_CLICKED, nullptr);
        pump();
    }

    CHECK_EQ(lua.eval("assert(fired == 3, 'fired ' .. fired)"), ERROR_NONE);
}

TEST_CASE("the handler receives its widget") {
    const UiRuntime lua;

    // One function can then serve several widgets
    CHECK_EQ(lua.eval(
        "button = lvgl.Button()\n"
        "got_self = false\n"
        "button:on(lvgl.EVENT.CLICKED, function(w) got_self = (w == button) end)\n"), ERROR_NONE);

    lv_obj_send_event(lua.widget("button"), LV_EVENT_CLICKED, nullptr);
    pump();

    CHECK_EQ(lua.eval("assert(got_self, 'handler did not receive its widget')"), ERROR_NONE);
}

TEST_CASE("on returns the widget so calls chain") {
    const UiRuntime lua;

    const auto* code =
        "local b = lvgl.Button()\n"
        "local returned = b:on(lvgl.EVENT.CLICKED, function() end)\n"
        "assert(returned == b)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("registering twice replaces rather than stacking") {
    const UiRuntime lua;

    CHECK_EQ(lua.eval(
        "button = lvgl.Button()\n"
        "count = 0\n"
        "button:on(lvgl.EVENT.CLICKED, function() count = count + 1 end)\n"
        "button:on(lvgl.EVENT.CLICKED, function() count = count + 10 end)\n"), ERROR_NONE);

    lv_obj_send_event(lua.widget("button"), LV_EVENT_CLICKED, nullptr);
    pump();

    // 10, not 11: the second registration replaced the first
    CHECK_EQ(lua.eval("assert(count == 10, 'count is ' .. count)"), ERROR_NONE);
}

TEST_CASE("different events on one widget are independent") {
    const UiRuntime lua;

    CHECK_EQ(lua.eval(
        "button = lvgl.Button()\n"
        "clicked, pressed = 0, 0\n"
        "button:on(lvgl.EVENT.CLICKED, function() clicked = clicked + 1 end)\n"
        "button:on(lvgl.EVENT.PRESSED, function() pressed = pressed + 1 end)\n"), ERROR_NONE);

    lv_obj_t* button = lua.widget("button");
    lv_obj_send_event(button, LV_EVENT_CLICKED, nullptr);
    lv_obj_send_event(button, LV_EVENT_PRESSED, nullptr);
    pump();

    CHECK_EQ(lua.eval("assert(clicked == 1 and pressed == 1, clicked .. '/' .. pressed)"), ERROR_NONE);
}

TEST_CASE("off stops a handler firing") {
    const UiRuntime lua;

    CHECK_EQ(lua.eval(
        "button = lvgl.Button()\n"
        "count = 0\n"
        "button:on(lvgl.EVENT.CLICKED, function() count = count + 1 end)\n"), ERROR_NONE);

    lv_obj_t* button = lua.widget("button");
    lv_obj_send_event(button, LV_EVENT_CLICKED, nullptr);
    pump();

    CHECK_EQ(lua.eval("button:off(lvgl.EVENT.CLICKED)"), ERROR_NONE);

    lv_obj_send_event(button, LV_EVENT_CLICKED, nullptr);
    pump();

    CHECK_EQ(lua.eval("assert(count == 1, 'count is ' .. count)"), ERROR_NONE);
}

TEST_CASE("a handler that errors does not take the runtime down") {
    const UiRuntime lua;

    CHECK_EQ(lua.eval(
        "button = lvgl.Button()\n"
        "button:on(lvgl.EVENT.CLICKED, function() error('boom') end)\n"), ERROR_NONE);

    lv_obj_send_event(lua.widget("button"), LV_EVENT_CLICKED, nullptr);
    pump();

    // The error has nowhere to propagate - LVGL called us - so it is swallowed
    CHECK_EQ(lua.eval("assert(true)"), ERROR_NONE);
}

TEST_CASE("a handler may delete its own widget") {
    const UiRuntime lua;

    // Only safe because dispatch is deferred: deleting mid-dispatch would corrupt LVGL's
    // event loop.
    CHECK_EQ(lua.eval(
        "button = lvgl.Button()\n"
        "button:on(lvgl.EVENT.CLICKED, function(w) w:delete() end)\n"), ERROR_NONE);

    lv_obj_send_event(lua.widget("button"), LV_EVENT_CLICKED, nullptr);
    pump();

    CHECK_EQ(lua.eval("assert(not button:is_valid(), 'widget survived')"), ERROR_NONE);
}

TEST_CASE("a widget deleted between event and dispatch is skipped") {
    const UiRuntime lua;

    CHECK_EQ(lua.eval(
        "button = lvgl.Button()\n"
        "fired = false\n"
        "button:on(lvgl.EVENT.CLICKED, function() fired = true end)\n"), ERROR_NONE);

    // Queue the dispatch, then delete before it runs
    lv_obj_send_event(lua.widget("button"), LV_EVENT_CLICKED, nullptr);
    CHECK_EQ(lua.eval("button:delete()"), ERROR_NONE);
    pump();

    CHECK_EQ(lua.eval("assert(fired == false, 'handler ran for a deleted widget')"), ERROR_NONE);
}

TEST_CASE("a queued event is cancelled when its runtime goes away") {
    LvglFixture lvgl;

    auto* runtime = lua_runtime_alloc();
    lua_bindings_open(runtime);
    luavgl_bindings_open(runtime, lvgl.screen);

    CHECK_EQ(lua_runtime_eval(runtime,
        "button = lvgl.Button()\n"
        "button:on(lvgl.EVENT.CLICKED, function() end)\n", "test"), ERROR_NONE);

    auto* state = static_cast<lua_State*>(lua_runtime_get_state(runtime));
    lua_getglobal(state, "button");
    auto* handle = static_cast<lv_obj_t**>(lua_touserdata(state, -1));
    REQUIRE(handle != nullptr);
    lv_obj_t* button = *handle;
    lua_pop(state, 1);

    // Press, then tear down before the queue is serviced
    lv_obj_send_event(button, LV_EVENT_CLICKED, nullptr);

    luavgl_bindings_close(runtime);
    lua_runtime_free(runtime);

    lv_obj_delete(lvgl.screen);
    lvgl.screen = nullptr;

    pump();
    pump();
}

TEST_CASE("on requires a function") {
    const UiRuntime lua;
    CHECK_NE(lua.eval("lvgl.Button():on(lvgl.EVENT.CLICKED, 'nope')"), ERROR_NONE);
    CHECK_NE(lua.eval("lvgl.Button():on(lvgl.EVENT.CLICKED)"), ERROR_NONE);
}

// Timers ------------------------------------------------------------------------------------

TEST_CASE("a timer fires after its period") {
    const UiRuntime lua;

    CHECK_EQ(lua.eval(
        "ticks = 0\n"
        "timer = lvgl.Timer{ period = 50, callback = function() ticks = ticks + 1 end }\n"),
        ERROR_NONE);

    CHECK_EQ(lua.eval("assert(ticks == 0, 'fired early')"), ERROR_NONE);

    pump(60);
    CHECK_EQ(lua.eval("assert(ticks == 1, 'ticks is ' .. ticks)"), ERROR_NONE);
}

TEST_CASE("a timer repeats") {
    const UiRuntime lua;

    CHECK_EQ(lua.eval(
        "ticks = 0\n"
        "timer = lvgl.Timer{ period = 20, callback = function() ticks = ticks + 1 end }\n"),
        ERROR_NONE);

    for (int i = 0; i < 3; i++) {
        pump(25);
    }

    CHECK_EQ(lua.eval("assert(ticks >= 3, 'ticks is ' .. ticks)"), ERROR_NONE);
}

TEST_CASE("repeat_count makes a one-shot") {
    const UiRuntime lua;

    CHECK_EQ(lua.eval(
        "ticks = 0\n"
        "timer = lvgl.Timer{ period = 20, repeat_count = 1,\n"
        "                    callback = function() ticks = ticks + 1 end }\n"), ERROR_NONE);

    for (int i = 0; i < 4; i++) {
        pump(25);
    }

    CHECK_EQ(lua.eval("assert(ticks == 1, 'ticks is ' .. ticks)"), ERROR_NONE);
}

TEST_CASE("a timer can be paused and resumed") {
    const UiRuntime lua;

    CHECK_EQ(lua.eval(
        "ticks = 0\n"
        "timer = lvgl.Timer{ period = 20, callback = function() ticks = ticks + 1 end }\n"
        "timer:pause()\n"), ERROR_NONE);

    pump(50);
    CHECK_EQ(lua.eval("assert(ticks == 0, 'ran while paused')"), ERROR_NONE);

    CHECK_EQ(lua.eval("timer:resume()"), ERROR_NONE);
    pump(50);
    CHECK_EQ(lua.eval("assert(ticks > 0, 'did not resume')"), ERROR_NONE);
}

TEST_CASE("a timer can start paused") {
    const UiRuntime lua;

    CHECK_EQ(lua.eval(
        "ticks = 0\n"
        "timer = lvgl.Timer{ period = 20, paused = true,\n"
        "                    callback = function() ticks = ticks + 1 end }\n"), ERROR_NONE);

    pump(50);
    CHECK_EQ(lua.eval("assert(ticks == 0, 'ran while paused')"), ERROR_NONE);
}

TEST_CASE("deleting a timer stops it") {
    const UiRuntime lua;

    CHECK_EQ(lua.eval(
        "ticks = 0\n"
        "timer = lvgl.Timer{ period = 20, callback = function() ticks = ticks + 1 end }\n"
        "timer:delete()\n"
        "assert(not timer:is_valid())\n"), ERROR_NONE);

    pump(60);
    CHECK_EQ(lua.eval("assert(ticks == 0, 'ran after delete')"), ERROR_NONE);
}

TEST_CASE("using a deleted timer raises") {
    const UiRuntime lua;

    const auto* code =
        "local t = lvgl.Timer{ period = 20, callback = function() end }\n"
        "t:delete()\n"
        "t:pause()";
    CHECK_NE(lua.eval(code), ERROR_NONE);
    CHECK(lua.error().find("deleted") != std::string::npos);
}

TEST_CASE("deleting a timer twice is harmless") {
    const UiRuntime lua;

    const auto* code =
        "local t = lvgl.Timer{ period = 20, callback = function() end }\n"
        "t:delete()\n"
        "t:delete()";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("a timer requires a callback and a positive period") {
    const UiRuntime lua;
    CHECK_NE(lua.eval("lvgl.Timer{ period = 20 }"), ERROR_NONE);
    CHECK_NE(lua.eval("lvgl.Timer{ period = 0, callback = function() end }"), ERROR_NONE);
    CHECK_NE(lua.eval("lvgl.Timer{ period = -5, callback = function() end }"), ERROR_NONE);
    CHECK_NE(lua.eval("lvgl.Timer('not a table')"), ERROR_NONE);
}

TEST_CASE("a timer whose handle is collected keeps running") {
    const UiRuntime lua;

    // A script that starts a repeating timer and keeps no reference still expects it to
    // run, so __gc must not stop it.
    CHECK_EQ(lua.eval(
        "ticks = 0\n"
        "do\n"
        "  local t = lvgl.Timer{ period = 20, callback = function() ticks = ticks + 1 end }\n"
        "end\n"
        "collectgarbage('collect')\n"
        "collectgarbage('collect')\n"), ERROR_NONE);

    pump(50);
    CHECK_EQ(lua.eval("assert(ticks > 0, 'timer was collected away')"), ERROR_NONE);
}

/**
 * The sharpest lifetime hazard in the binding: a timer has no widget to anchor it, so
 * nothing else would notice its runtime had gone - it would simply keep firing into freed
 * memory.
 */
TEST_CASE("timers are stopped when their runtime closes") {
    LvglFixture lvgl;

    auto* runtime = lua_runtime_alloc();
    lua_bindings_open(runtime);
    luavgl_bindings_open(runtime, lvgl.screen);

    CHECK_EQ(lua_runtime_eval(runtime,
        "for i = 1, 5 do\n"
        "  lvgl.Timer{ period = 10, callback = function() end }\n"
        "end\n"
        "collectgarbage('collect')\n", "test"), ERROR_NONE);

    luavgl_bindings_close(runtime);
    lua_runtime_free(runtime);

    lv_obj_delete(lvgl.screen);
    lvgl.screen = nullptr;

    // Any surviving timer fires here, against a freed state
    for (int i = 0; i < 5; i++) {
        pump(15);
    }
}

TEST_CASE("set_period changes the interval") {
    const UiRuntime lua;

    const auto* code =
        "local t = lvgl.Timer{ period = 1000, callback = function() end }\n"
        "local returned = t:set_period(50)\n"
        "assert(returned == t)\n"
        "t:delete()";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("ready makes a timer fire on the next cycle") {
    const UiRuntime lua;

    CHECK_EQ(lua.eval(
        "ticks = 0\n"
        "timer = lvgl.Timer{ period = 10000, callback = function() ticks = ticks + 1 end }\n"
        "timer:ready()\n"), ERROR_NONE);

    pump(1);
    CHECK_EQ(lua.eval("assert(ticks == 1, 'ticks is ' .. ticks)"), ERROR_NONE);
}

TEST_CASE("tostring reflects whether a timer is alive") {
    const UiRuntime lua;

    const auto* code =
        "local t = lvgl.Timer{ period = 50, callback = function() end }\n"
        "assert(tostring(t):find('lvgl.Timer'), tostring(t))\n"
        "t:delete()\n"
        "assert(tostring(t):find('deleted'), tostring(t))";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}
