#include "doctest.h"

#include <lua/bindings.h>
#include <lua/runtime.h>
#include <string>

namespace {

struct BoundRuntime {
    LuaRuntime* runtime = lua_runtime_alloc();

    BoundRuntime() { lua_bindings_open(runtime); }
    ~BoundRuntime() { lua_runtime_free(runtime); }

    error_t eval(const char* code) const { return lua_runtime_eval(runtime, code, "test"); }
    std::string error() const { return lua_runtime_get_error(runtime); }
};

}

TEST_CASE("the time table is present") {
    const BoundRuntime lua;
    CHECK_EQ(lua.eval("assert(type(tactility.time) == 'table')"), ERROR_NONE);
}

TEST_CASE("uptime is a non-negative number that advances") {
    const BoundRuntime lua;

    const auto* code =
        "local first = tactility.time.uptime()\n"
        "assert(type(first) == 'number' and first >= 0)\n"
        "tactility.time.sleep(5)\n"
        "local second = tactility.time.uptime()\n"
        "assert(second >= first, ('%d < %d'):format(second, first))";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("uptime_micros advances and is finer than milliseconds") {
    const BoundRuntime lua;

    const auto* code =
        "local first = tactility.time.uptime_micros()\n"
        "assert(type(first) == 'number' and first >= 0)\n"
        "local second = tactility.time.uptime_micros()\n"
        "assert(second >= first)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("sleep blocks for at least the requested time") {
    const BoundRuntime lua;

    const auto* code =
        "local start = tactility.time.uptime()\n"
        "tactility.time.sleep(50)\n"
        "local elapsed = tactility.time.uptime() - start\n"
        "assert(elapsed >= 40, 'only slept ' .. elapsed .. 'ms')";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("sleep rejects a negative duration") {
    const BoundRuntime lua;
    CHECK_NE(lua.eval("tactility.time.sleep(-1)"), ERROR_NONE);
    CHECK(lua.error().find("negative") != std::string::npos);
}

TEST_CASE("sleep(0) is allowed and returns immediately") {
    const BoundRuntime lua;
    CHECK_EQ(lua.eval("tactility.time.sleep(0)"), ERROR_NONE);
}

TEST_CASE("now returns a plausible timestamp, or says the clock is unset") {
    const BoundRuntime lua;

    const auto* code =
        "local seconds, reason = tactility.time.now()\n"
        "if seconds == nil then\n"
        "  assert(reason == 'clock not set', tostring(reason))\n"
        "else\n"
        "  assert(seconds > 1000000000, 'implausibly early: ' .. seconds)\n"
        "end";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("date returns a table shaped like os.date('*t')") {
    const BoundRuntime lua;

    // A fixed timestamp keeps this independent of the host clock. Only the fields'
    // ranges are checked, since the result is in local time and the zone is unknown.
    const auto* code =
        "local t = tactility.time.date(1700000000)\n"
        "assert(type(t) == 'table')\n"
        "assert(t.year >= 2023 and t.year <= 2024, 'year ' .. t.year)\n"
        "assert(t.month >= 1 and t.month <= 12, 'month ' .. t.month)\n"
        "assert(t.day >= 1 and t.day <= 31)\n"
        "assert(t.hour >= 0 and t.hour <= 23)\n"
        "assert(t.min >= 0 and t.min <= 59)\n"
        "assert(t.sec >= 0 and t.sec <= 60)\n"
        "assert(t.wday >= 1 and t.wday <= 7, 'wday ' .. t.wday)\n"
        "assert(t.yday >= 1 and t.yday <= 366)\n"
        "assert(type(t.isdst) == 'boolean')";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("date uses 1-based months and weekdays, matching os.date") {
    const BoundRuntime lua;

    // The C struct is 0-based for both; a binding that forgot to adjust would show it here
    const auto* code =
        "local ours = tactility.time.date(1700000000)\n"
        "local theirs = os.date('*t', 1700000000)\n"
        "assert(ours.year == theirs.year)\n"
        "assert(ours.month == theirs.month, ('%d vs %d'):format(ours.month, theirs.month))\n"
        "assert(ours.day == theirs.day)\n"
        "assert(ours.wday == theirs.wday, ('%d vs %d'):format(ours.wday, theirs.wday))\n"
        "assert(ours.yday == theirs.yday)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("date with no argument uses the current time") {
    const BoundRuntime lua;

    const auto* code =
        "local t = tactility.time.date()\n"
        "assert(type(t) == 'table')\n"
        "assert(type(t.year) == 'number')";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("a non-numeric timestamp raises") {
    const BoundRuntime lua;
    CHECK_NE(lua.eval("tactility.time.date({})"), ERROR_NONE);
    CHECK_NE(lua.eval("tactility.time.sleep('soon')"), ERROR_NONE);
}
