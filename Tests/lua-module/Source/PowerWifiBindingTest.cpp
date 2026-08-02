#include "doctest.h"

#include <lua/bindings.h>
#include <lua/runtime.h>
#include <string>

/**
 * These run on the simulator, which may have no power supply and only a mock wifi. So the
 * assertions are written to hold either way: when hardware is absent every call must
 * return nil plus a reason rather than raising or crashing, and when it is present the
 * values must be in range. That "absent" path is the one real devices hit too - a board
 * without a battery is normal.
 */

namespace {

struct BoundRuntime {
    LuaRuntime* runtime = lua_runtime_alloc();

    BoundRuntime() { lua_bindings_open(runtime); }
    ~BoundRuntime() { lua_runtime_free(runtime); }

    error_t eval(const char* code) const { return lua_runtime_eval(runtime, code, "test"); }
    std::string error() const { return lua_runtime_get_error(runtime); }
};

}

// Power ---------------------------------------------------------------------------------

TEST_CASE("the power table is present") {
    const BoundRuntime lua;
    CHECK_EQ(lua.eval("assert(type(tactility.power) == 'table')"), ERROR_NONE);
}

TEST_CASE("is_available answers without raising") {
    const BoundRuntime lua;
    CHECK_EQ(lua.eval("assert(type(tactility.power.is_available()) == 'boolean')"), ERROR_NONE);
}

TEST_CASE("every power reader returns a number or nil plus a reason") {
    const BoundRuntime lua;

    const auto* code =
        "for _, name in ipairs({'capacity', 'voltage', 'current'}) do\n"
        "  local value, reason = tactility.power[name]()\n"
        "  if value == nil then\n"
        "    assert(type(reason) == 'string' and #reason > 0, name .. ': no reason given')\n"
        "  else\n"
        "    assert(type(value) == 'number', name .. ' returned ' .. type(value))\n"
        "  end\n"
        "end";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("capacity is a percentage when present") {
    const BoundRuntime lua;

    const auto* code =
        "local pct = tactility.power.capacity()\n"
        "if pct ~= nil then\n"
        "  assert(pct >= 0 and pct <= 100, 'capacity out of range: ' .. pct)\n"
        "end";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("is_charging is a boolean when present") {
    const BoundRuntime lua;

    const auto* code =
        "local charging, reason = tactility.power.is_charging()\n"
        "if charging == nil then\n"
        "  assert(type(reason) == 'string')\n"
        "else\n"
        "  assert(type(charging) == 'boolean')\n"
        "end";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("capabilities reports booleans for every feature") {
    const BoundRuntime lua;

    const auto* code =
        "local caps, reason = tactility.power.capabilities()\n"
        "if caps == nil then\n"
        "  assert(type(reason) == 'string')\n"
        "else\n"
        "  for _, key in ipairs({'capacity', 'voltage', 'current', 'is_charging',\n"
        "                        'charge_control', 'quick_charge', 'power_off'}) do\n"
        "    assert(type(caps[key]) == 'boolean', key .. ' is ' .. type(caps[key]))\n"
        "  end\n"
        "end";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("a reader agrees with what capabilities claims") {
    const BoundRuntime lua;

    // If capabilities says capacity is unsupported, reading it must fail rather than
    // returning a made-up number - and vice versa.
    const auto* code =
        "local caps = tactility.power.capabilities()\n"
        "if caps ~= nil then\n"
        "  local value = tactility.power.capacity()\n"
        "  if caps.capacity then\n"
        "    assert(value ~= nil, 'claimed supported but read failed')\n"
        "  else\n"
        "    assert(value == nil, 'claimed unsupported but returned ' .. tostring(value))\n"
        "  end\n"
        "end";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("set_allowed_to_charge requires a boolean") {
    const BoundRuntime lua;
    CHECK_NE(lua.eval("tactility.power.set_allowed_to_charge()"), ERROR_NONE);
    CHECK_NE(lua.eval("tactility.power.set_allowed_to_charge('yes')"), ERROR_NONE);
    CHECK_NE(lua.eval("tactility.power.set_allowed_to_charge(1)"), ERROR_NONE);
}

TEST_CASE("power_off is not exposed") {
    const BoundRuntime lua;
    // Deliberately unbound: a script must not be able to kill the device outright
    CHECK_EQ(lua.eval("assert(tactility.power.power_off == nil)"), ERROR_NONE);
}

TEST_CASE("repeated power reads do not leak references") {
    const BoundRuntime lua;

    // Each call takes and releases a device reference; an unbalanced put would show up
    // as a refcount that never returns to zero, and a leaked get would eventually fail.
    const auto* code =
        "for i = 1, 200 do\n"
        "  tactility.power.is_available()\n"
        "  tactility.power.capacity()\n"
        "  tactility.power.capabilities()\n"
        "end";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

// Wifi ----------------------------------------------------------------------------------

TEST_CASE("the wifi table is present") {
    const BoundRuntime lua;
    CHECK_EQ(lua.eval("assert(type(tactility.wifi) == 'table')"), ERROR_NONE);
}

TEST_CASE("wifi is_available answers without raising") {
    const BoundRuntime lua;
    CHECK_EQ(lua.eval("assert(type(tactility.wifi.is_available()) == 'boolean')"), ERROR_NONE);
}

TEST_CASE("radio_state is one of the known names") {
    const BoundRuntime lua;

    const auto* code =
        "local state, reason = tactility.wifi.radio_state()\n"
        "if state == nil then\n"
        "  assert(type(reason) == 'string')\n"
        "else\n"
        "  local valid = { off = true, turning_on = true, on = true, turning_off = true, unknown = true }\n"
        "  assert(valid[state], 'unexpected radio state: ' .. tostring(state))\n"
        "end";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("station_state is one of the known names") {
    const BoundRuntime lua;

    const auto* code =
        "local state, reason = tactility.wifi.station_state()\n"
        "if state == nil then\n"
        "  assert(type(reason) == 'string')\n"
        "else\n"
        "  local valid = { disconnected = true, connecting = true, connected = true, unknown = true }\n"
        "  assert(valid[state], 'unexpected station state: ' .. tostring(state))\n"
        "end";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("connection details return a string or nil plus a reason") {
    const BoundRuntime lua;

    const auto* code =
        "for _, name in ipairs({'ip_address', 'ssid'}) do\n"
        "  local value, reason = tactility.wifi[name]()\n"
        "  if value == nil then\n"
        "    assert(type(reason) == 'string' and #reason > 0, name)\n"
        "  else\n"
        "    assert(type(value) == 'string', name .. ' returned ' .. type(value))\n"
        "  end\n"
        "end";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("rssi is a negative-ish number when connected") {
    const BoundRuntime lua;

    const auto* code =
        "local rssi, reason = tactility.wifi.rssi()\n"
        "if rssi == nil then\n"
        "  assert(type(reason) == 'string')\n"
        "else\n"
        "  assert(type(rssi) == 'number')\n"
        "  assert(rssi > -200 and rssi < 100, 'implausible rssi: ' .. rssi)\n"
        "end";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("is_scanning is a boolean or nil plus a reason") {
    const BoundRuntime lua;

    const auto* code =
        "local scanning, reason = tactility.wifi.is_scanning()\n"
        "if scanning == nil then\n"
        "  assert(type(reason) == 'string')\n"
        "else\n"
        "  assert(type(scanning) == 'boolean')\n"
        "end";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("scan_results is a well-formed array") {
    const BoundRuntime lua;

    const auto* code =
        "local results, reason = tactility.wifi.scan_results()\n"
        "if results == nil then\n"
        "  assert(type(reason) == 'string')\n"
        "else\n"
        "  assert(type(results) == 'table')\n"
        "  for _, ap in ipairs(results) do\n"
        "    assert(type(ap.ssid) == 'string')\n"
        "    assert(type(ap.rssi) == 'number')\n"
        "    assert(type(ap.channel) == 'number')\n"
        "    assert(type(ap.authentication) == 'string')\n"
        "    assert(type(ap.secured) == 'boolean')\n"
        "  end\n"
        "end";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("scan returns promptly rather than blocking") {
    const BoundRuntime lua;

    // Scanning takes seconds; the binding must start it and return, or the LVGL task
    // would stall for the duration once Lua apps draw.
    const auto* code =
        "local start = tactility.time.uptime()\n"
        "tactility.wifi.scan()\n"
        "local elapsed = tactility.time.uptime() - start\n"
        "assert(elapsed < 1000, 'scan blocked for ' .. elapsed .. 'ms')";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("connecting is not exposed") {
    const BoundRuntime lua;

    // Deliberately unbound until there is a story for how a script gets credentials
    const auto* code =
        "assert(tactility.wifi.connect == nil)\n"
        "assert(tactility.wifi.station_connect == nil)\n"
        "assert(tactility.wifi.disconnect == nil)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("repeated wifi reads do not leak references") {
    const BoundRuntime lua;

    const auto* code =
        "for i = 1, 200 do\n"
        "  tactility.wifi.is_available()\n"
        "  tactility.wifi.radio_state()\n"
        "  tactility.wifi.scan_results()\n"
        "end";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}
