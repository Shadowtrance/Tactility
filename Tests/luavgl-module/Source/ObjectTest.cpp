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
 * These drive LVGL headless: lv_init() plus a display with a dummy flush is enough to
 * create and lay out widgets, which is all the bindings touch. Nothing renders, so no
 * SDL and no window.
 */

namespace {

constexpr int32_t DISPLAY_WIDTH = 320;
constexpr int32_t DISPLAY_HEIGHT = 240;

lv_display_t* display = nullptr;
uint8_t* draw_buffer = nullptr;

void flush_nothing(lv_display_t* target, const lv_area_t*, uint8_t*) {
    lv_display_flush_ready(target);
}

/** Brings LVGL up once for the whole suite and hands out a fresh screen per test. */
struct LvglFixture {
    lv_obj_t* screen = nullptr;

    LvglFixture() {
        if (!lv_is_initialized()) {
            lv_init();

            display = lv_display_create(DISPLAY_WIDTH, DISPLAY_HEIGHT);
            REQUIRE(display != nullptr);

            const size_t buffer_size = DISPLAY_WIDTH * 40 * 2; // partial buffer, RGB565
            draw_buffer = new uint8_t[buffer_size];
            lv_display_set_buffers(display, draw_buffer, nullptr, buffer_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
            lv_display_set_flush_cb(display, flush_nothing);
        }

        // A per-test screen keeps widgets from one test out of the next
        screen = lv_obj_create(nullptr);
        REQUIRE(screen != nullptr);
    }

    ~LvglFixture() {
        if (screen != nullptr) {
            lv_obj_delete(screen);
        }
    }
};

/** A runtime with both the kernel and LVGL bindings opened against a test screen. */
struct UiRuntime {
    LvglFixture lvgl;
    LuaRuntime* runtime = lua_runtime_alloc();

    UiRuntime() {
        lua_bindings_open(runtime);
        luavgl_bindings_open(runtime, lvgl.screen);
    }

    ~UiRuntime() {
        // Detach before closing: the screen is torn down by ~LvglFixture afterwards, and
        // its widgets still carry delete callbacks pointing at this state.
        luavgl_bindings_close(runtime);
        lua_runtime_free(runtime);
    }

    error_t eval(const char* code) const { return lua_runtime_eval(runtime, code, "test"); }
    std::string error() const { return lua_runtime_get_error(runtime); }
};

}

/*
 * Note on geometry: LVGL does not compute sizes or positions until a layout pass runs, so
 * lv_obj_get_width() reports 0 immediately after a width is set. Tests that read geometry
 * back call obj:update_layout() first, which is what a script has to do too when it needs
 * the numbers before the next frame.
 */

TEST_CASE("the lvgl table is present") {
    const UiRuntime lua;
    CHECK_EQ(lua.eval("assert(type(lvgl) == 'table')"), ERROR_NONE);
    CHECK_EQ(lua.eval("assert(type(lvgl.Label) == 'function')"), ERROR_NONE);
    CHECK_EQ(lua.eval("assert(type(lvgl.ALIGN) == 'table')"), ERROR_NONE);
}

TEST_CASE("a runtime without the lvgl bindings has no lvgl table") {
    auto* runtime = lua_runtime_alloc();
    lua_bindings_open(runtime);

    CHECK_EQ(lua_runtime_eval(runtime, "assert(lvgl == nil)", "test"), ERROR_NONE);

    lua_runtime_free(runtime);
}

// Creation --------------------------------------------------------------------------------

TEST_CASE("widgets can be created and are valid") {
    const UiRuntime lua;

    const auto* code =
        "local obj = lvgl.Object()\n"
        "assert(obj ~= nil)\n"
        "assert(obj:is_valid())\n"
        "local label = lvgl.Label()\n"
        "assert(label:is_valid())\n"
        "local button = lvgl.Button()\n"
        "assert(button:is_valid())";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("a widget parents to the root by default") {
    const UiRuntime lua;

    const auto* code =
        "local label = lvgl.Label()\n"
        "assert(label:parent() == lvgl.root())";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("a widget can be given an explicit parent") {
    const UiRuntime lua;

    const auto* code =
        "local panel = lvgl.Object()\n"
        "local label = lvgl.Label(panel)\n"
        "assert(label:parent() == panel)\n"
        "assert(#panel:children() == 1)\n"
        "assert(panel:children()[1] == label)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("a widget can be created with properties in one call") {
    const UiRuntime lua;

    const auto* code =
        "local label = lvgl.Label{ text = 'hello', w = 100, h = 20 }\n"
        "assert(label:is_valid())\n"
        "local w, h = label:update_layout():size()\n"
        "assert(w == 100, 'width is ' .. w)\n"
        "assert(h == 20, 'height is ' .. h)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("a parent and properties can be given together") {
    const UiRuntime lua;

    const auto* code =
        "local panel = lvgl.Object()\n"
        "local label = lvgl.Label(panel, { text = 'in panel', w = 50 })\n"
        "assert(label:parent() == panel)\n"
        "local w = label:update_layout():size()\n"
        "assert(w == 50)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

// Identity --------------------------------------------------------------------------------

TEST_CASE("the same widget always yields the same handle") {
    const UiRuntime lua;

    // Wrapping a widget twice must not produce two handles, or == would be surprising
    const auto* code =
        "local panel = lvgl.Object()\n"
        "local label = lvgl.Label(panel)\n"
        "assert(label:parent() == panel)\n"
        "assert(panel:children()[1] == label)\n"
        "assert(rawequal(panel:children()[1], label), 'not the same userdata')";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

// Properties ------------------------------------------------------------------------------

TEST_CASE("set applies properties and returns self for chaining") {
    const UiRuntime lua;

    const auto* code =
        "local label = lvgl.Label()\n"
        "local returned = label:set{ text = 'chained', w = 80 }\n"
        "assert(returned == label)\n"
        "local w = label:update_layout():size()\n"
        "assert(w == 80)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("position and alignment apply") {
    const UiRuntime lua;

    const auto* code =
        "local obj = lvgl.Object{ w = 40, h = 40, x = 12, y = 34 }\n"
        "local x, y = obj:update_layout():position()\n"
        "assert(x == 12, 'x is ' .. x)\n"
        "assert(y == 34, 'y is ' .. y)\n"
        "obj:set{ align = lvgl.ALIGN.CENTER }";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("style properties apply without error") {
    const UiRuntime lua;

    const auto* code =
        "local obj = lvgl.Object{\n"
        "  w = 100, h = 50,\n"
        "  bg_color = 0xFF8800, bg_opa = lvgl.OPA.COVER,\n"
        "  border_color = 0x000000, border_width = 2,\n"
        "  radius = 8, pad_all = 4, opa = lvgl.OPA.COVER,\n"
        "}\n"
        "assert(obj:is_valid())";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("an unknown property raises rather than being ignored") {
    const UiRuntime lua;

    // Silently ignoring a typo would leave a script looking correct while doing nothing
    CHECK_NE(lua.eval("lvgl.Object{ colour = 0xFF0000 }"), ERROR_NONE);
    CHECK(lua.error().find("colour") != std::string::npos);
}

TEST_CASE("text on a widget that has none raises") {
    const UiRuntime lua;

    // LVGL's setters do not check the class, so writing text through a plain object
    // would corrupt memory rather than fail
    CHECK_NE(lua.eval("lvgl.Object{ text = 'nope' }"), ERROR_NONE);
    CHECK(lua.error().find("text") != std::string::npos);
}

TEST_CASE("percentages and content sizing are available") {
    const UiRuntime lua;

    const auto* code =
        "local obj = lvgl.Object{ w = lvgl.PCT(50), h = lvgl.SIZE_CONTENT }\n"
        "assert(obj:is_valid())";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("flex layout applies") {
    const UiRuntime lua;

    const auto* code =
        "local column = lvgl.Object{ w = 200, h = 200, flex_flow = lvgl.FLEX_FLOW.COLUMN }\n"
        "lvgl.Label(column, { text = 'one' })\n"
        "lvgl.Label(column, { text = 'two' })\n"
        "assert(#column:children() == 2)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

// Lifetime --------------------------------------------------------------------------------

TEST_CASE("a deleted widget reports itself invalid") {
    const UiRuntime lua;

    const auto* code =
        "local label = lvgl.Label()\n"
        "assert(label:is_valid())\n"
        "label:delete()\n"
        "assert(not label:is_valid())";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("using a deleted widget raises instead of touching freed memory") {
    const UiRuntime lua;

    const auto* code =
        "local label = lvgl.Label()\n"
        "label:delete()\n"
        "label:set{ text = 'gone' }";
    CHECK_NE(lua.eval(code), ERROR_NONE);
    CHECK(lua.error().find("deleted") != std::string::npos);
}

TEST_CASE("deleting a parent invalidates handles on its children") {
    const UiRuntime lua;

    // This is the case that makes LVGL and Lua lifetimes hard: LVGL frees children with
    // the parent, and the script is still holding handles on them.
    const auto* code =
        "local panel = lvgl.Object()\n"
        "local child = lvgl.Label(panel)\n"
        "local grandchild = lvgl.Label(child)\n"
        "assert(child:is_valid() and grandchild:is_valid())\n"
        "panel:delete()\n"
        "assert(not child:is_valid(), 'child survived its parent')\n"
        "assert(not grandchild:is_valid(), 'grandchild survived')";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("a child handle raises after its parent is deleted") {
    const UiRuntime lua;

    const auto* code =
        "local panel = lvgl.Object()\n"
        "local child = lvgl.Label(panel)\n"
        "panel:delete()\n"
        "child:size()";
    CHECK_NE(lua.eval(code), ERROR_NONE);
    CHECK(lua.error().find("deleted") != std::string::npos);
}

TEST_CASE("tostring reflects whether the widget is alive") {
    const UiRuntime lua;

    const auto* code =
        "local label = lvgl.Label()\n"
        "assert(tostring(label):find('lvgl.Object'), tostring(label))\n"
        "label:delete()\n"
        "assert(tostring(label):find('deleted'), tostring(label))";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("collecting a handle does not delete the widget") {
    const UiRuntime lua;

    // LVGL owns the widget; a dropped handle must not take it down, or a script that
    // stops referencing a label would see it vanish from the screen.
    const auto* code =
        "local panel = lvgl.Object()\n"
        "do\n"
        "  local temporary = lvgl.Label(panel, { text = 'still here' })\n"
        "end\n"
        "collectgarbage('collect')\n"
        "collectgarbage('collect')\n"
        "assert(#panel:children() == 1, 'widget was collected away')\n"
        "assert(panel:children()[1]:is_valid())";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("many widgets can be created and deleted without leaking lua memory") {
    const UiRuntime lua;

    CHECK_EQ(lua.eval("collectgarbage('collect')"), ERROR_NONE);
    const size_t baseline = lua_runtime_get_memory_used(lua.runtime);

    const auto* code =
        "for i = 1, 200 do\n"
        "  local label = lvgl.Label{ text = 'row ' .. i }\n"
        "  label:delete()\n"
        "end\n"
        "collectgarbage('collect')\n"
        "collectgarbage('collect')";
    CHECK_EQ(lua.eval(code), ERROR_NONE);

    // The registry holds handles weakly, so deleted widgets' handles must be collectable
    CHECK(lua_runtime_get_memory_used(lua.runtime) < baseline + 65536);
}

TEST_CASE("widgets can outlive the runtime that created them") {
    LvglFixture lvgl;

    auto* runtime = lua_runtime_alloc();
    lua_bindings_open(runtime);
    luavgl_bindings_open(runtime, lvgl.screen);

    CHECK_EQ(lua_runtime_eval(runtime, "held = lvgl.Label{ text = 'left behind' }", "test"), ERROR_NONE);

    // This is what an app does on exit: close the script while its UI is still on screen.
    // Every wrapped widget holds a delete callback pointing at this lua_State, so they
    // must be detached before it goes away.
    luavgl_bindings_close(runtime);
    lua_runtime_free(runtime);

    // Tearing the tree down now must not call back into the freed state
    lv_obj_delete(lvgl.screen);
    lvgl.screen = nullptr;
}

TEST_CASE("detaching twice, or without any widgets, is harmless") {
    LvglFixture lvgl;

    auto* runtime = lua_runtime_alloc();
    lua_bindings_open(runtime);
    luavgl_bindings_open(runtime, lvgl.screen);

    luavgl_bindings_close(runtime);
    luavgl_bindings_close(runtime);

    lua_runtime_free(runtime);
}

TEST_CASE("detaching a runtime leaves another runtime's widgets working") {
    LvglFixture lvgl;

    auto* first = lua_runtime_alloc();
    lua_bindings_open(first);
    luavgl_bindings_open(first, lvgl.screen);

    auto* second = lua_runtime_alloc();
    lua_bindings_open(second);
    luavgl_bindings_open(second, lvgl.screen);

    CHECK_EQ(lua_runtime_eval(first, "mine = lvgl.Label{ text = 'first' }", "first"), ERROR_NONE);
    CHECK_EQ(lua_runtime_eval(second, "mine = lvgl.Label{ text = 'second' }", "second"), ERROR_NONE);

    // Detaching one must only remove its own callbacks, matched on the state pointer
    luavgl_bindings_close(first);
    lua_runtime_free(first);

    // The second runtime's handle must still track deletion correctly
    CHECK_EQ(lua_runtime_eval(second, "assert(mine:is_valid())", "second"), ERROR_NONE);
    CHECK_EQ(lua_runtime_eval(second, "mine:delete() assert(not mine:is_valid())", "second"), ERROR_NONE);

    luavgl_bindings_close(second);
    lua_runtime_free(second);
}

// Screen ----------------------------------------------------------------------------------

TEST_CASE("resolution is reported") {
    const UiRuntime lua;

    const auto* code =
        "assert(lvgl.HOR_RES() == 320, 'got ' .. lvgl.HOR_RES())\n"
        "assert(lvgl.VER_RES() == 240, 'got ' .. lvgl.VER_RES())";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("flags can be set and read") {
    const UiRuntime lua;

    const auto* code =
        "local obj = lvgl.Object()\n"
        "assert(not obj:has_flag(lvgl.FLAG.HIDDEN))\n"
        "obj:add_flag(lvgl.FLAG.HIDDEN)\n"
        "assert(obj:has_flag(lvgl.FLAG.HIDDEN))\n"
        "obj:remove_flag(lvgl.FLAG.HIDDEN)\n"
        "assert(not obj:has_flag(lvgl.FLAG.HIDDEN))";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("center places a widget in its parent") {
    const UiRuntime lua;

    const auto* code =
        "local obj = lvgl.Object{ w = 40, h = 40 }\n"
        "local returned = obj:center()\n"
        "assert(returned == obj)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

// Toolbar ---------------------------------------------------------------------------------

TEST_CASE("a toolbar can be created and titled") {
    const UiRuntime lua;

    const auto* code =
        "local bar = lvgl.toolbar.create('Title')\n"
        "assert(bar:is_valid())\n"
        "lvgl.toolbar.set_title(bar, 'Changed')";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("a toolbar nav action stores a callable") {
    const UiRuntime lua;

    const auto* code =
        "local bar = lvgl.toolbar.create('Title')\n"
        "local returned = lvgl.toolbar.set_nav_action(bar, 'close', function() end)\n"
        "assert(returned == bar)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("a toolbar nav action requires a function") {
    const UiRuntime lua;

    const auto* code =
        "local bar = lvgl.toolbar.create('Title')\n"
        "lvgl.toolbar.set_nav_action(bar, 'close', 'not a function')";
    CHECK_NE(lua.eval(code), ERROR_NONE);
}

TEST_CASE("toolbar text actions can be added and cleared") {
    const UiRuntime lua;

    const auto* code =
        "local bar = lvgl.toolbar.create('Title')\n"
        "local button = lvgl.toolbar.add_text_action(bar, 'Go', function() end)\n"
        "assert(button:is_valid())\n"
        "lvgl.toolbar.clear_actions(bar)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("a fresh toolbar already has a working close action") {
    const UiRuntime lua;

    // The C++ toolbar installs a default nav action, so a Lua app that sets none still
    // gets a close button. Setting one is an override, not a requirement.
    const auto* code =
        "local bar = lvgl.toolbar.create('Title')\n"
        "assert(bar:is_valid())\n"
        "assert(#bar:children() > 0, 'toolbar has no children')";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("a toolbar can be given an explicit parent") {
    const UiRuntime lua;

    const auto* code =
        "local panel = lvgl.Object()\n"
        "local bar = lvgl.toolbar.create(panel, 'Title')\n"
        "assert(bar:parent() == panel)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("set_nav_action takes a callback with or without an icon") {
    const UiRuntime lua;

    const auto* code =
        "local bar = lvgl.toolbar.create('Title')\n"
        "lvgl.toolbar.set_nav_action(bar, function() end)\n"
        "lvgl.toolbar.set_nav_action(bar, lvgl.SYMBOL.CLOSE, function() end)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("the built-in symbols are non-empty strings") {
    const UiRuntime lua;

    // These are symbol-font strings, not image paths. Passing an arbitrary string where
    // an icon is expected renders nothing, which is how the first toolbar looked blank.
    const auto* code =
        "assert(type(lvgl.SYMBOL) == 'table')\n"
        "for _, name in ipairs({'CLOSE', 'OK', 'SETTINGS', 'WIFI', 'HOME'}) do\n"
        "  local s = lvgl.SYMBOL[name]\n"
        "  assert(type(s) == 'string' and #s > 0, name .. ' is ' .. tostring(s))\n"
        "end";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("a toolbar callback surviving into a freed runtime does not crash") {
    LvglFixture lvgl;

    auto* runtime = lua_runtime_alloc();
    lua_bindings_open(runtime);
    luavgl_bindings_open(runtime, lvgl.screen);

    CHECK_EQ(lua_runtime_eval(runtime,
        "bar = lvgl.toolbar.create('Title')\n"
        "lvgl.toolbar.set_nav_action(bar, function() end)\n", "test"), ERROR_NONE);

    // The toolbar lives on the app's screen, which the loader tears down *after* the app
    // is destroyed and its runtime freed. Its click callback points at this lua_State, so
    // without detaching it the teardown - or any later press - runs against freed memory.
    // On device that surfaced as an illegal instruction inside luaM_malloc_.
    luavgl_bindings_close(runtime);
    lua_runtime_free(runtime);

    lv_obj_delete(lvgl.screen);
    lvgl.screen = nullptr;
}

TEST_CASE("a toolbar action actually runs its lua callback") {
    const UiRuntime lua;

    CHECK_EQ(lua.eval(
        "bar = lvgl.toolbar.create('Title')\n"
        "fired = false\n"
        "button = lvgl.toolbar.add_text_action(bar, 'Go', function() fired = true end)\n"),
        ERROR_NONE);

    // Fire it the way a tap would, through LVGL rather than by calling the binding
    lua_State* state = static_cast<lua_State*>(lua_runtime_get_state(lua.runtime));
    lua_getglobal(state, "button");
    auto* handle = static_cast<lv_obj_t**>(lua_touserdata(state, -1));
    REQUIRE(handle != nullptr);
    lua_pop(state, 1);

    // The toolbar registers on SHORT_CLICKED, not CLICKED. The handler is queued via
    // lv_async_call (see on_action), so the timers have to run before it fires.
    lv_obj_send_event(*handle, LV_EVENT_SHORT_CLICKED, nullptr);
    lv_timer_handler();

    CHECK_EQ(lua.eval("assert(fired == true, 'callback did not run')"), ERROR_NONE);
}

/**
 * A handler that stops its app must defer the teardown.
 *
 * Doing it inline frees the runtime while lua_pcall is still on the stack, and Lua reads
 * the call info *after* the C function returns - a use-after-free inside Lua itself, which
 * no amount of care in the binding can prevent. On device it appeared as an illegal
 * instruction in luaM_malloc_, the allocator pointer having gone with the unloaded ELF.
 *
 * This mirrors what the app does with lv_async_call: record the intent during the event,
 * act on it afterwards.
 */
TEST_CASE("a toolbar callback can request teardown, deferred until after the event") {
    LvglFixture lvgl;

    auto* runtime = lua_runtime_alloc();
    lua_bindings_open(runtime);
    luavgl_bindings_open(runtime, lvgl.screen);

    static bool stop_requested = false;
    stop_requested = false;

    lua_State* state = static_cast<lua_State*>(lua_runtime_get_state(runtime));

    lua_pushcfunction(state, [](lua_State*) {
        stop_requested = true; // the real app calls lv_async_call here
        return 0;
    });
    lua_setglobal(state, "request_stop");

    CHECK_EQ(lua_runtime_eval(runtime,
        "bar = lvgl.toolbar.create('Title')\n"
        "button = lvgl.toolbar.add_text_action(bar, 'Close', function() request_stop() end)\n",
        "test"), ERROR_NONE);

    lua_getglobal(state, "button");
    auto* handle = static_cast<lv_obj_t**>(lua_touserdata(state, -1));
    REQUIRE(handle != nullptr);
    lv_obj_t* button = *handle;
    lua_pop(state, 1);

    lv_obj_send_event(button, LV_EVENT_SHORT_CLICKED, nullptr);
    lv_timer_handler();
    CHECK(stop_requested);

    // Now the event has unwound, so tearing down is safe - as it is for the real app
    luavgl_bindings_close(runtime);
    lua_runtime_free(runtime);

    lv_obj_delete(lvgl.screen);
    lvgl.screen = nullptr;
}

/**
 * The nav action is installed on an internal child of the toolbar, not on the toolbar
 * object. Tracking the toolbar instead left the real callback attached after teardown,
 * pointing at a freed lua_State - the crash that survived two attempted fixes because
 * every earlier test only ever checked the widget the binding *returned*.
 */
TEST_CASE("no lua callback survives detach anywhere in the toolbar tree") {
    LvglFixture lvgl;

    auto* runtime = lua_runtime_alloc();
    lua_bindings_open(runtime);
    luavgl_bindings_open(runtime, lvgl.screen);

    CHECK_EQ(lua_runtime_eval(runtime,
        "bar = lvgl.toolbar.create('Title')\n"
        "lvgl.toolbar.set_nav_action(bar, function() end)\n"
        "lvgl.toolbar.add_text_action(bar, 'Go', function() end)\n", "test"), ERROR_NONE);

    lua_State* state = static_cast<lua_State*>(lua_runtime_get_state(runtime));

    lua_getglobal(state, "bar");
    auto* handle = static_cast<lv_obj_t**>(lua_touserdata(state, -1));
    REQUIRE(handle != nullptr);
    lv_obj_t* toolbar = *handle;
    lua_pop(state, 1);

    // Walk the whole subtree, not just the toolbar object, since that is where the
    // callbacks actually live.
    CHECK(luavgl_test_has_event_callback(toolbar, state));

    luavgl_bindings_close(runtime);

    CHECK_FALSE(luavgl_test_has_event_callback(toolbar, state));

    lua_runtime_free(runtime);

    lv_obj_delete(lvgl.screen);
    lvgl.screen = nullptr;
}

/**
 * A widget whose Lua handle has been collected is still alive and still carries a delete
 * callback pointing at this state.
 *
 * The object map holds handles *weakly*, so such a widget looks absent from it - and using
 * that map as the liveness test meant detach skipped exactly these widgets. The callback
 * survived, and firing it during the loader's lv_obj_clean() dereferenced a freed
 * lua_State. LVGL's own lv_obj_is_valid() is the correct test.
 */
TEST_CASE("widgets whose handles were collected are still detached") {
    LvglFixture lvgl;

    auto* runtime = lua_runtime_alloc();
    lua_bindings_open(runtime);
    luavgl_bindings_open(runtime, lvgl.screen);

    // Create widgets, then drop every reference so their handles become collectable
    CHECK_EQ(lua_runtime_eval(runtime,
        "do\n"
        "  local panel = lvgl.Object()\n"
        "  for i = 1, 20 do lvgl.Label(panel, { text = 'row ' .. i }) end\n"
        "end\n"
        "collectgarbage('collect')\n"
        "collectgarbage('collect')\n", "test"), ERROR_NONE);

    // The widgets are still on screen even though Lua no longer references them
    CHECK(lv_obj_get_child_count(lvgl.screen) > 0);

    luavgl_bindings_close(runtime);
    lua_runtime_free(runtime);

    // This is what the loader does when showing the next app. Every delete callback must
    // already be gone, or it reaches into the freed state.
    lv_obj_clean(lvgl.screen);

    lv_obj_delete(lvgl.screen);
    lvgl.screen = nullptr;
}

TEST_CASE("a full app teardown sequence does not crash") {
    LvglFixture lvgl;

    auto* runtime = lua_runtime_alloc();
    lua_bindings_open(runtime);
    luavgl_bindings_open(runtime, lvgl.screen);

    // Approximates the demo app: a toolbar plus a scrolling list of rows, with most
    // handles left to the collector rather than kept in locals.
    CHECK_EQ(lua_runtime_eval(runtime,
        "local bar = lvgl.toolbar.create('Title')\n"
        "lvgl.toolbar.set_nav_action(bar, function() end)\n"
        "local page = lvgl.Object{ flex_flow = lvgl.FLEX_FLOW.COLUMN }\n"
        "for i = 1, 12 do\n"
        "  local line = lvgl.Object(page, { flex_flow = lvgl.FLEX_FLOW.ROW })\n"
        "  lvgl.Label(line, { text = 'label ' .. i })\n"
        "  lvgl.Label(line, { text = 'value ' .. i })\n"
        "end\n"
        "collectgarbage('collect')\n", "test"), ERROR_NONE);

    // onDestroy, then the loader tearing the screen down
    luavgl_bindings_close(runtime);
    lua_runtime_free(runtime);

    lv_obj_clean(lvgl.screen);
    lv_obj_delete(lvgl.screen);
    lvgl.screen = nullptr;
}

/**
 * Stopping an app is asynchronous, so the close button stays alive and clickable while
 * the teardown runs. A second press must not re-enter the handler: on device that ran
 * lua_pcall against a runtime already being destroyed and jumped to address zero.
 */
TEST_CASE("a toolbar action fires at most once") {
    const UiRuntime lua;

    CHECK_EQ(lua.eval(
        "bar = lvgl.toolbar.create('Title')\n"
        "count = 0\n"
        "button = lvgl.toolbar.add_text_action(bar, 'Go', function() count = count + 1 end)\n"),
        ERROR_NONE);

    lua_State* state = static_cast<lua_State*>(lua_runtime_get_state(lua.runtime));
    lua_getglobal(state, "button");
    auto* handle = static_cast<lv_obj_t**>(lua_touserdata(state, -1));
    REQUIRE(handle != nullptr);
    lv_obj_t* button = *handle;
    lua_pop(state, 1);

    lv_obj_send_event(button, LV_EVENT_SHORT_CLICKED, nullptr);
    lv_timer_handler();
    CHECK_EQ(lua.eval("assert(count == 1, 'count is ' .. count)"), ERROR_NONE);

    // Impatient second and third press, as happens when an app takes a moment to close
    lv_obj_send_event(button, LV_EVENT_SHORT_CLICKED, nullptr);
    lv_obj_send_event(button, LV_EVENT_SHORT_CLICKED, nullptr);
    lv_timer_handler();
    CHECK_EQ(lua.eval("assert(count == 1, 'fired again: ' .. count)"), ERROR_NONE);
}

TEST_CASE("a nav action also fires at most once") {
    const UiRuntime lua;

    CHECK_EQ(lua.eval(
        "bar = lvgl.toolbar.create('Title')\n"
        "count = 0\n"
        "lvgl.toolbar.set_nav_action(bar, function() count = count + 1 end)\n"),
        ERROR_NONE);

    lua_State* state = static_cast<lua_State*>(lua_runtime_get_state(lua.runtime));
    lua_getglobal(state, "bar");
    auto* handle = static_cast<lv_obj_t**>(lua_touserdata(state, -1));
    REQUIRE(handle != nullptr);
    lv_obj_t* toolbar = *handle;
    lua_pop(state, 1);

    // The nav callback lives on an internal child, so find it the way dispatch would
    lv_obj_t* nav = luavgl_test_find_event_target(toolbar, state);
    REQUIRE(nav != nullptr);

    lv_obj_send_event(nav, LV_EVENT_SHORT_CLICKED, nullptr);
    lv_timer_handler();
    CHECK_EQ(lua.eval("assert(count == 1, 'count is ' .. count)"), ERROR_NONE);

    lv_obj_send_event(nav, LV_EVENT_SHORT_CLICKED, nullptr);
    lv_timer_handler();
    CHECK_EQ(lua.eval("assert(count == 1, 'fired again: ' .. count)"), ERROR_NONE);
}

/**
 * The press that closes an app leaves its handler queued on the LVGL async list, and the
 * teardown it triggers can complete before that queue is next serviced. The queued call
 * must then do nothing rather than reach into a freed runtime.
 */
TEST_CASE("a queued action is cancelled when its runtime goes away") {
    LvglFixture lvgl;

    auto* runtime = lua_runtime_alloc();
    lua_bindings_open(runtime);
    luavgl_bindings_open(runtime, lvgl.screen);

    CHECK_EQ(lua_runtime_eval(runtime,
        "bar = lvgl.toolbar.create('Title')\n"
        "button = lvgl.toolbar.add_text_action(bar, 'Close', function() end)\n", "test"),
        ERROR_NONE);

    lua_State* state = static_cast<lua_State*>(lua_runtime_get_state(runtime));
    lua_getglobal(state, "button");
    auto* handle = static_cast<lv_obj_t**>(lua_touserdata(state, -1));
    REQUIRE(handle != nullptr);
    lv_obj_t* button = *handle;
    lua_pop(state, 1);

    // Press, but do NOT service the async queue - the handler is still pending
    lv_obj_send_event(button, LV_EVENT_SHORT_CLICKED, nullptr);

    // The app tears down first, exactly as it does when the handler asked it to
    luavgl_bindings_close(runtime);
    lua_runtime_free(runtime);

    lv_obj_delete(lvgl.screen);
    lvgl.screen = nullptr;

    // Now the queue runs. The cancelled entry must return without touching the state.
    lv_timer_handler();
    lv_timer_handler();
}

/**
 * The toolbar ships with a default close handler, and lvgl_toolbar_set_nav_action() *adds*
 * a callback rather than replacing one. Without removing the default first both fire: the
 * app closes, and then the script's handler closes whatever ended up on top of the stack
 * afterwards - which on device meant the launcher's app list vanished too.
 */
TEST_CASE("setting a nav action replaces the default rather than stacking on it") {
    const UiRuntime lua;

    CHECK_EQ(lua.eval(
        "bar = lvgl.toolbar.create('Title')\n"
        "count = 0\n"
        "lvgl.toolbar.set_nav_action(bar, function() count = count + 1 end)\n"),
        ERROR_NONE);

    lua_State* state = static_cast<lua_State*>(lua_runtime_get_state(lua.runtime));
    lua_getglobal(state, "bar");
    auto* handle = static_cast<lv_obj_t**>(lua_touserdata(state, -1));
    REQUIRE(handle != nullptr);
    lv_obj_t* toolbar = *handle;
    lua_pop(state, 1);

    lv_obj_t* nav = luavgl_test_find_event_target(toolbar, state);
    REQUIRE(nav != nullptr);

    // Exactly one handler on the button: the script's, not the default as well
    CHECK_EQ(lv_obj_get_event_count(nav), 1);
}

TEST_CASE("pad_row and pad_column are settable") {
    const UiRuntime lua;

    // Needed to match a native screen's layout: pad_all leaves the flex gap untouched
    const auto* code =
        "local column = lvgl.Object{ flex_flow = lvgl.FLEX_FLOW.COLUMN,\n"
        "                            pad_all = 0, pad_row = 0, pad_column = 0 }\n"
        "assert(column:is_valid())";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

// clean() ------------------------------------------------------------------------------------

TEST_CASE("clean deletes every child but keeps the object") {
    const UiRuntime lua;

    const auto* code =
        "local panel = lvgl.Object{ w = 100, h = 100 }\n"
        "for i = 1, 5 do lvgl.Label(panel, { text = 'row ' .. i }) end\n"
        "assert(#panel:children() == 5, 'got ' .. #panel:children())\n"
        "panel:clean()\n"
        "assert(#panel:children() == 0, 'got ' .. #panel:children())\n"
        "assert(panel:is_valid())\n";

    REQUIRE(lua.eval(code) == ERROR_NONE);
}

TEST_CASE("clean returns the object so calls chain") {
    const UiRuntime lua;

    const auto* code =
        "local panel = lvgl.Object{ w = 10, h = 10 }\n"
        "assert(panel:clean() == panel)\n";

    REQUIRE(lua.eval(code) == ERROR_NONE);
}

/**
 * The hazard: a handle a script still holds must not survive its widget being cleaned
 * away, or the next use is a dangling pointer.
 */
TEST_CASE("a handle to a cleaned child reports itself deleted") {
    const UiRuntime lua;

    const auto* code =
        "local panel = lvgl.Object{ w = 100, h = 100 }\n"
        "local child = lvgl.Label(panel, { text = 'x' })\n"
        "assert(child:is_valid())\n"
        "panel:clean()\n"
        "assert(not child:is_valid(), 'handle outlived its widget')\n"
        "local ok = pcall(function() child:set{ text = 'y' } end)\n"
        "assert(not ok, 'using a cleaned widget should raise')\n";

    REQUIRE(lua.eval(code) == ERROR_NONE);
}

TEST_CASE("clean reaches grandchildren too") {
    const UiRuntime lua;

    const auto* code =
        "local panel = lvgl.Object{ w = 100, h = 100 }\n"
        "local middle = lvgl.Object(panel, { w = 50, h = 50 })\n"
        "local leaf = lvgl.Label(middle, { text = 'deep' })\n"
        "panel:clean()\n"
        "assert(not middle:is_valid())\n"
        "assert(not leaf:is_valid(), 'grandchild handle outlived its widget')\n";

    REQUIRE(lua.eval(code) == ERROR_NONE);
}

/** Rebuilding into a cleaned container is the view-swap the editor relies on. */
TEST_CASE("a cleaned object can be rebuilt into") {
    const UiRuntime lua;

    const auto* code =
        "local content = lvgl.Object{ w = 100, h = 100,\n"
        "                             flex_flow = lvgl.FLEX_FLOW.COLUMN }\n"
        "for pass = 1, 3 do\n"
        "    content:clean()\n"
        "    for i = 1, 4 do lvgl.Label(content, { text = 'pass ' .. pass }) end\n"
        "    assert(#content:children() == 4, 'got ' .. #content:children())\n"
        "end\n"
        "content:update_layout()\n";

    REQUIRE(lua.eval(code) == ERROR_NONE);
}

/** Cleaning a widget that carries handlers must not leave them dispatching. */
TEST_CASE("clean removes children that had event handlers") {
    const UiRuntime lua;

    const auto* code =
        "local panel = lvgl.Object{ w = 100, h = 100 }\n"
        "local button = lvgl.Button(panel, { w = 40, h = 20 })\n"
        "button:on(lvgl.EVENT.CLICKED, function() end)\n"
        "panel:clean()\n"
        "assert(not button:is_valid())\n";

    REQUIRE(lua.eval(code) == ERROR_NONE);
}
