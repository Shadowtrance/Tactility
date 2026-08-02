#include "doctest.h"

#include <lua/bindings.h>
#include <lua/runtime.h>
#include <string>
#include <tactility/device.h>
#include <tactility/driver.h>
#include <tactility/module.h>

namespace {

/** A runtime with the bindings opened, torn down automatically. */
struct BoundRuntime {
    LuaRuntime* runtime = lua_runtime_alloc();

    BoundRuntime() { lua_bindings_open(runtime); }
    ~BoundRuntime() { lua_runtime_free(runtime); }

    error_t eval(const char* code) const { return lua_runtime_eval(runtime, code, "test"); }
    std::string error() const { return lua_runtime_get_error(runtime); }
};

Module test_module = {
    .name = "lua_bindings_test_module",
    .start = nullptr,
    .stop = nullptr
};

int start_device(Device*) { return ERROR_NONE; }
int stop_device(Device*) { return ERROR_NONE; }

/** device_start() refuses a device with no driver, so even a do-nothing one is needed. */
Driver test_driver = {
    .name = "lua_bindings_test_driver",
    .compatible = (const char*[]) { "lua-bindings-test", nullptr },
    .start_device = start_device,
    .stop_device = stop_device,
    .api = nullptr,
    .device_type = nullptr,
    .owner = &test_module,
    .internal = nullptr,
};

/**
 * Registers the shared test driver for as long as any device needs it. Driver
 * registration is global, so it is reference counted rather than done per device.
 */
struct DriverRegistration {
    static inline int count = 0;

    DriverRegistration() {
        if (count++ == 0) {
            REQUIRE_EQ(driver_construct_add(&test_driver), ERROR_NONE);
        }
    }

    ~DriverRegistration() {
        if (--count == 0) {
            driver_remove_destruct(&test_driver);
        }
    }
};

/** A device that is constructed, added and started for the duration of the scope. */
struct ScopedDevice {
    DriverRegistration registration;
    Device device;

    explicit ScopedDevice(const char* name, Device* parent = nullptr)
        : device { .name = name, .parent = parent } {
        REQUIRE_EQ(device_construct(&device), ERROR_NONE);
        device_set_driver(&device, &test_driver);
        REQUIRE_EQ(device_add(&device), ERROR_NONE);
        REQUIRE_EQ(device_start(&device), ERROR_NONE);
    }

    ~ScopedDevice() {
        device_stop(&device);
        device_remove(&device);
        device_destruct(&device);
    }
};

}

TEST_CASE("the tactility table is present after opening the bindings") {
    const BoundRuntime lua;
    CHECK_EQ(lua.eval("assert(type(tactility) == 'table')"), ERROR_NONE);
    CHECK_EQ(lua.eval("assert(type(tactility.log) == 'table')"), ERROR_NONE);
    CHECK_EQ(lua.eval("assert(type(tactility.device) == 'table')"), ERROR_NONE);
}

TEST_CASE("a runtime without bindings has no access to the device") {
    auto* runtime = lua_runtime_alloc();
    REQUIRE(runtime != nullptr);

    CHECK_EQ(lua_runtime_eval(runtime, "assert(tactility == nil)", "test"), ERROR_NONE);

    lua_runtime_free(runtime);
}

// Logging -----------------------------------------------------------------------------

TEST_CASE("every log level is callable") {
    const BoundRuntime lua;
    CHECK_EQ(lua.eval("tactility.log.error('an error')"), ERROR_NONE);
    CHECK_EQ(lua.eval("tactility.log.warning('a warning')"), ERROR_NONE);
    CHECK_EQ(lua.eval("tactility.log.info('some info')"), ERROR_NONE);
    CHECK_EQ(lua.eval("tactility.log.debug('a debug line')"), ERROR_NONE);
    CHECK_EQ(lua.eval("tactility.log.verbose('a verbose line')"), ERROR_NONE);
}

TEST_CASE("logging accepts an explicit tag") {
    const BoundRuntime lua;
    CHECK_EQ(lua.eval("tactility.log.info('my-tag', 'tagged message')"), ERROR_NONE);
}

TEST_CASE("logging accepts non-string values") {
    const BoundRuntime lua;
    CHECK_EQ(lua.eval("tactility.log.info(42)"), ERROR_NONE);
    CHECK_EQ(lua.eval("tactility.log.info(true)"), ERROR_NONE);
    CHECK_EQ(lua.eval("tactility.log.info(nil)"), ERROR_NONE);
}

TEST_CASE("logging leaves the lua stack balanced") {
    const BoundRuntime lua;
    // A leak of one value per call would accumulate and eventually overflow
    CHECK_EQ(lua.eval("for i = 1, 200 do tactility.log.info('x') end"), ERROR_NONE);
}

// Device discovery --------------------------------------------------------------------

TEST_CASE("device.list returns a table of names including a started device") {
    ScopedDevice scoped("lua_test_list");
    const BoundRuntime lua;

    const auto* code =
        "local names = tactility.device.list()\n"
        "assert(type(names) == 'table')\n"
        "local found = false\n"
        "for _, name in ipairs(names) do if name == 'lua_test_list' then found = true end end\n"
        "assert(found, 'device not in list')";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("device.find returns a handle for a started device") {
    ScopedDevice scoped("lua_test_find");
    const BoundRuntime lua;

    const auto* code =
        "local d = tactility.device.find('lua_test_find')\n"
        "assert(d ~= nil, 'device not found')\n"
        "assert(d:name() == 'lua_test_find')\n"
        "d:release()";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("device.find returns nil and a reason for an unknown device") {
    const BoundRuntime lua;

    const auto* code =
        "local d, reason = tactility.device.find('definitely_not_a_device')\n"
        "assert(d == nil)\n"
        "assert(type(reason) == 'string' and #reason > 0)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("a handle reports readiness and child count") {
    ScopedDevice scoped("lua_test_props");
    const BoundRuntime lua;

    const auto* code =
        "local d = tactility.device.find('lua_test_props')\n"
        "assert(d:is_ready() == true)\n"
        "assert(d:child_count() == 0)\n"
        "assert(type(d:children()) == 'table')\n"
        "d:release()";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("tostring on a handle names the device") {
    ScopedDevice scoped("lua_test_tostring");
    const BoundRuntime lua;

    const auto* code =
        "local d = tactility.device.find('lua_test_tostring')\n"
        "assert(tostring(d):find('lua_test_tostring'), tostring(d))\n"
        "d:release()\n"
        "assert(tostring(d):find('released'), tostring(d))";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

// Reference counting ------------------------------------------------------------------

TEST_CASE("releasing a handle drops the reference, so the device can be destructed") {
    Device device = { .name = "lua_test_release" };
    DriverRegistration registration;
    REQUIRE_EQ(device_construct(&device), ERROR_NONE);
    device_set_driver(&device, &test_driver);
    REQUIRE_EQ(device_add(&device), ERROR_NONE);
    REQUIRE_EQ(device_start(&device), ERROR_NONE);

    {
        const BoundRuntime lua;
        CHECK_EQ(lua.eval("local d = tactility.device.find('lua_test_release') d:release()"), ERROR_NONE);
    }

    CHECK_EQ(device_stop(&device), ERROR_NONE);
    CHECK_EQ(device_remove(&device), ERROR_NONE);
    // A reference left over from the script would make this ERROR_RESOURCE_BUSY
    CHECK_EQ(device_destruct(&device), ERROR_NONE);
}

/**
 * Reference counts are observed through device_destruct(), not device_stop(): stop only
 * unbinds the driver and ignores outstanding references, while destruct is the operation
 * that refuses to run while any reference is held.
 */
TEST_CASE("an unreleased handle keeps the device referenced until collection") {
    Device device = { .name = "lua_test_gc" };
    DriverRegistration registration;
    REQUIRE_EQ(device_construct(&device), ERROR_NONE);
    device_set_driver(&device, &test_driver);
    REQUIRE_EQ(device_add(&device), ERROR_NONE);
    REQUIRE_EQ(device_start(&device), ERROR_NONE);

    {
        const BoundRuntime lua;

        // Deliberately no release(): the handle is dropped and left to the collector
        CHECK_EQ(lua.eval("held = tactility.device.find('lua_test_gc')"), ERROR_NONE);

        CHECK_EQ(device_stop(&device), ERROR_NONE);
        CHECK_EQ(device_remove(&device), ERROR_NONE);
        CHECK_EQ(device_destruct(&device), ERROR_RESOURCE_BUSY);

        CHECK_EQ(lua.eval("held = nil collectgarbage('collect')"), ERROR_NONE);

        // __gc must have run device_put, so the reference is gone
        CHECK_EQ(device_destruct(&device), ERROR_NONE);
    }
}

TEST_CASE("closing a runtime releases handles the script never released") {
    Device device = { .name = "lua_test_close" };
    DriverRegistration registration;
    REQUIRE_EQ(device_construct(&device), ERROR_NONE);
    device_set_driver(&device, &test_driver);
    REQUIRE_EQ(device_add(&device), ERROR_NONE);
    REQUIRE_EQ(device_start(&device), ERROR_NONE);

    auto* runtime = lua_runtime_alloc();
    lua_bindings_open(runtime);

    CHECK_EQ(lua_runtime_eval(runtime, "held = tactility.device.find('lua_test_close')", "test"), ERROR_NONE);

    CHECK_EQ(device_stop(&device), ERROR_NONE);
    CHECK_EQ(device_remove(&device), ERROR_NONE);
    CHECK_EQ(device_destruct(&device), ERROR_RESOURCE_BUSY);

    // This is the case that matters for apps: a script exits without cleaning up, and
    // closing its runtime must not strand a reference on the device.
    lua_runtime_free(runtime);

    CHECK_EQ(device_destruct(&device), ERROR_NONE);
}

TEST_CASE("releasing twice is harmless") {
    ScopedDevice scoped("lua_test_double_release");
    const BoundRuntime lua;

    // A second device_put would corrupt the count, so release() must be idempotent
    const auto* code =
        "local d = tactility.device.find('lua_test_double_release')\n"
        "d:release()\n"
        "d:release()";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("using a released handle raises rather than touching freed state") {
    ScopedDevice scoped("lua_test_use_after_release");
    const BoundRuntime lua;

    const auto* code =
        "local d = tactility.device.find('lua_test_use_after_release')\n"
        "d:release()\n"
        "d:name()";
    CHECK_EQ(lua.eval(code), ERROR_UNDEFINED);
    CHECK(lua.error().find("released") != std::string::npos);
}

/** Runs `code` against a fresh device, then checks every reference was handed back. */
static void check_no_stranded_references(const char* device_name, const char* code) {
    Device device = { .name = device_name };
    DriverRegistration registration;
    REQUIRE_EQ(device_construct(&device), ERROR_NONE);
    device_set_driver(&device, &test_driver);
    REQUIRE_EQ(device_add(&device), ERROR_NONE);
    REQUIRE_EQ(device_start(&device), ERROR_NONE);

    {
        const BoundRuntime lua;
        CHECK_EQ(lua.eval(code), ERROR_NONE);
    }

    CHECK_EQ(device_stop(&device), ERROR_NONE);
    CHECK_EQ(device_remove(&device), ERROR_NONE);
    CHECK_EQ(device_destruct(&device), ERROR_NONE);
}

TEST_CASE("many lookups in a loop do not strand references") {
    check_no_stranded_references(
        "lua_test_loop",
        "for i = 1, 100 do\n"
        "  local d = tactility.device.find('lua_test_loop')\n"
        "  assert(d ~= nil)\n"
        "  d:release()\n"
        "end"
    );
}

TEST_CASE("handles dropped without release are reclaimed in a loop") {
    // No release() at all: this only balances if __gc runs as handles go unreachable
    check_no_stranded_references(
        "lua_test_loop_gc",
        "for i = 1, 100 do\n"
        "  local d = tactility.device.find('lua_test_loop_gc')\n"
        "  assert(d ~= nil)\n"
        "end\n"
        "collectgarbage('collect')"
    );
}

// Parent / child ----------------------------------------------------------------------

TEST_CASE("a child reports its parent and appears in the parent's children") {
    ScopedDevice parent("lua_test_parent");
    ScopedDevice child("lua_test_child", &parent.device);

    const BoundRuntime lua;

    const auto* code =
        "local c = tactility.device.find('lua_test_child')\n"
        "local p = c:parent()\n"
        "assert(p ~= nil, 'no parent')\n"
        "assert(p:name() == 'lua_test_parent')\n"
        "assert(p:child_count() == 1)\n"
        "local names = p:children()\n"
        "assert(names[1] == 'lua_test_child')\n"
        "p:release()\n"
        "c:release()";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("a device without a parent returns nil") {
    ScopedDevice scoped("lua_test_no_parent");
    const BoundRuntime lua;

    const auto* code =
        "local d = tactility.device.find('lua_test_no_parent')\n"
        "assert(d:parent() == nil)\n"
        "d:release()";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

// Isolation ---------------------------------------------------------------------------

TEST_CASE("the handle metatable is not reachable from a script") {
    ScopedDevice scoped("lua_test_metatable");
    const BoundRuntime lua;

    // getmetatable must not expose __gc, or a script could unhook reference release
    const auto* code =
        "local d = tactility.device.find('lua_test_metatable')\n"
        "assert(getmetatable(d) == false)\n"
        "d:release()";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("bindings in one runtime do not leak into another") {
    const BoundRuntime bound;
    auto* plain = lua_runtime_alloc();
    REQUIRE(plain != nullptr);

    CHECK_EQ(lua_runtime_eval(plain, "assert(tactility == nil)", "plain"), ERROR_NONE);

    lua_runtime_free(plain);
}
