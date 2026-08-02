#include "doctest.h"

#include <cstdio>
#include <cstring>
#include <lua/runtime.h>
#include <string>

/** Runs a chunk that writes its result to a file, and returns what it wrote. */
static std::string run_and_read_output(const char* code) {
    const auto* path = "lua_test_output.txt";
    std::remove(path);

    auto* runtime = lua_runtime_alloc();
    REQUIRE(runtime != nullptr);

    const auto script = std::string("local f = io.open('") + path + "', 'w') " + code + " f:close()";
    CHECK_EQ(lua_runtime_eval(runtime, script.c_str(), "test"), ERROR_NONE);
    lua_runtime_free(runtime);

    std::string content;
    if (auto* file = std::fopen(path, "r")) {
        char buffer[256];
        while (std::fgets(buffer, sizeof(buffer), file) != nullptr) {
            content += buffer;
        }
        std::fclose(file);
    }
    std::remove(path);
    return content;
}

TEST_CASE("a runtime can be allocated and freed") {
    auto* runtime = lua_runtime_alloc();
    REQUIRE(runtime != nullptr);
    CHECK_EQ(std::strlen(lua_runtime_get_error(runtime)), 0);
    lua_runtime_free(runtime);
}

TEST_CASE("lua_runtime_eval runs a valid chunk") {
    auto* runtime = lua_runtime_alloc();
    REQUIRE(runtime != nullptr);
    CHECK_EQ(lua_runtime_eval(runtime, "local x = 1 + 1", "test"), ERROR_NONE);
    lua_runtime_free(runtime);
}

TEST_CASE("lua_runtime_eval actually evaluates, rather than only parsing") {
    CHECK_EQ(run_and_read_output("f:write(tostring(6 * 7))"), "42");
}

TEST_CASE("the standard library is available") {
    CHECK_EQ(run_and_read_output("f:write(string.upper('ok'), ' ', tostring(math.floor(3.7)))"), "OK 3");
}

TEST_CASE("lua_runtime_eval reports a syntax error without running anything") {
    auto* runtime = lua_runtime_alloc();
    REQUIRE(runtime != nullptr);

    CHECK_EQ(lua_runtime_eval(runtime, "this is not lua", "test"), ERROR_INVALID_ARGUMENT);

    // The message should name the chunk, so a failing app is identifiable in the log
    const std::string error = lua_runtime_get_error(runtime);
    CHECK(error.find("test") != std::string::npos);
    CHECK_FALSE(error.empty());

    lua_runtime_free(runtime);
}

TEST_CASE("lua_runtime_eval reports a runtime error with a traceback") {
    auto* runtime = lua_runtime_alloc();
    REQUIRE(runtime != nullptr);

    CHECK_EQ(lua_runtime_eval(runtime, "error('boom')", "test"), ERROR_UNDEFINED);

    const std::string error = lua_runtime_get_error(runtime);
    CHECK(error.find("boom") != std::string::npos);
    // add_traceback() should have appended the stack, not just the bare message
    CHECK(error.find("stack traceback") != std::string::npos);

    lua_runtime_free(runtime);
}

TEST_CASE("an error raised from a nested function still yields a traceback") {
    auto* runtime = lua_runtime_alloc();
    REQUIRE(runtime != nullptr);

    const auto* code =
        "local function inner() error('deep') end\n"
        "local function outer() inner() end\n"
        "outer()";
    CHECK_EQ(lua_runtime_eval(runtime, code, "nested"), ERROR_UNDEFINED);

    const std::string error = lua_runtime_get_error(runtime);
    CHECK(error.find("deep") != std::string::npos);
    CHECK(error.find("stack traceback") != std::string::npos);

    lua_runtime_free(runtime);
}

TEST_CASE("a failed run leaves the runtime usable") {
    auto* runtime = lua_runtime_alloc();
    REQUIRE(runtime != nullptr);

    CHECK_NE(lua_runtime_eval(runtime, "error('first')", "test"), ERROR_NONE);
    // A stale stack from the failed call would surface here
    CHECK_EQ(lua_runtime_eval(runtime, "local x = 1", "test"), ERROR_NONE);

    lua_runtime_free(runtime);
}

TEST_CASE("the error message is cleared by a subsequent successful run") {
    auto* runtime = lua_runtime_alloc();
    REQUIRE(runtime != nullptr);

    CHECK_NE(lua_runtime_eval(runtime, "error('boom')", "test"), ERROR_NONE);
    CHECK_FALSE(std::string(lua_runtime_get_error(runtime)).empty());

    CHECK_EQ(lua_runtime_eval(runtime, "local x = 1", "test"), ERROR_NONE);
    CHECK_EQ(std::strlen(lua_runtime_get_error(runtime)), 0);

    lua_runtime_free(runtime);
}

TEST_CASE("runtimes are isolated from each other") {
    auto* first = lua_runtime_alloc();
    auto* second = lua_runtime_alloc();
    REQUIRE(first != nullptr);
    REQUIRE(second != nullptr);

    CHECK_EQ(lua_runtime_eval(first, "leaked = 'from first'", "first"), ERROR_NONE);

    // The second runtime must not see the first's global, or apps could reach each other
    CHECK_EQ(lua_runtime_eval(second, "assert(leaked == nil)", "second"), ERROR_NONE);

    lua_runtime_free(first);
    lua_runtime_free(second);
}

TEST_CASE("memory accounting tracks the lua heap") {
    auto* runtime = lua_runtime_alloc();
    REQUIRE(runtime != nullptr);

    const size_t baseline = lua_runtime_get_memory_used(runtime);
    CHECK(baseline > 0); // the standard library is already loaded

    CHECK_EQ(lua_runtime_eval(runtime, "big = {} for i = 1, 5000 do big[i] = i end", "test"), ERROR_NONE);
    CHECK(lua_runtime_get_memory_used(runtime) > baseline);

    // Dropping the reference and collecting should hand the memory back through our allocator
    CHECK_EQ(lua_runtime_eval(runtime, "big = nil collectgarbage('collect')", "test"), ERROR_NONE);
    CHECK(lua_runtime_get_memory_used(runtime) < baseline + 10000);

    lua_runtime_free(runtime);
}

TEST_CASE("lua_runtime_eval_file runs a script from disk and receives its argument") {
    const auto* script_path = "lua_test_script.lua";
    const auto* output_path = "lua_test_script_output.txt";
    std::remove(output_path);

    {
        auto* file = std::fopen(script_path, "w");
        REQUIRE(file != nullptr);
        // `...` is how an app receives its install directory
        std::fputs(
            "local dir = ...\n"
            "local f = io.open('lua_test_script_output.txt', 'w')\n"
            "f:write(dir)\n"
            "f:close()\n",
            file
        );
        std::fclose(file);
    }

    auto* runtime = lua_runtime_alloc();
    REQUIRE(runtime != nullptr);
    CHECK_EQ(lua_runtime_eval_file(runtime, script_path, "/data/app/demo"), ERROR_NONE);
    lua_runtime_free(runtime);

    std::string content;
    if (auto* file = std::fopen(output_path, "r")) {
        char buffer[256];
        if (std::fgets(buffer, sizeof(buffer), file) != nullptr) {
            content = buffer;
        }
        std::fclose(file);
    }

    CHECK_EQ(content, "/data/app/demo");

    std::remove(script_path);
    std::remove(output_path);
}

TEST_CASE("lua_runtime_eval_file fails cleanly when the script is missing") {
    auto* runtime = lua_runtime_alloc();
    REQUIRE(runtime != nullptr);

    CHECK_EQ(lua_runtime_eval_file(runtime, "/nonexistent/nope.lua", nullptr), ERROR_INVALID_ARGUMENT);
    CHECK_FALSE(std::string(lua_runtime_get_error(runtime)).empty());

    lua_runtime_free(runtime);
}

// Lifecycle callbacks -----------------------------------------------------------------------

TEST_CASE("lua_runtime_has_function finds global functions only") {
    auto* runtime = lua_runtime_alloc();
    REQUIRE(runtime != nullptr);

    REQUIRE_EQ(lua_runtime_eval(runtime, "function on_show() end\nnot_a_function = 42\n", "test"), ERROR_NONE);

    CHECK(lua_runtime_has_function(runtime, "on_show"));
    CHECK_FALSE(lua_runtime_has_function(runtime, "on_hide"));
    // A global that exists but is not callable must not be reported as a function, or the
    // caller would go on to call it and get a confusing error instead of skipping it.
    CHECK_FALSE(lua_runtime_has_function(runtime, "not_a_function"));

    lua_runtime_free(runtime);
}

TEST_CASE("lua_runtime_call invokes a global function") {
    auto* runtime = lua_runtime_alloc();
    REQUIRE(runtime != nullptr);

    REQUIRE_EQ(lua_runtime_eval(runtime, "calls = 0\nfunction on_show() calls = calls + 1 end\n", "test"), ERROR_NONE);

    CHECK_EQ(lua_runtime_call(runtime, "on_show"), ERROR_NONE);
    CHECK_EQ(lua_runtime_call(runtime, "on_show"), ERROR_NONE);

    // Read the counter back through a function, since eval discards results
    REQUIRE_EQ(lua_runtime_eval(runtime, "assert(calls == 2, 'expected 2, got ' .. calls)\n", "test"), ERROR_NONE);

    lua_runtime_free(runtime);
}

TEST_CASE("lua_runtime_call reports an error raised by the script") {
    auto* runtime = lua_runtime_alloc();
    REQUIRE(runtime != nullptr);

    REQUIRE_EQ(lua_runtime_eval(runtime, "function on_show() error('boom') end\n", "test"), ERROR_NONE);

    CHECK_NE(lua_runtime_call(runtime, "on_show"), ERROR_NONE);
    CHECK(std::string(lua_runtime_get_error(runtime)).find("boom") != std::string::npos);

    lua_runtime_free(runtime);
}

TEST_CASE("lua_runtime_call fails on a name that is not a function") {
    auto* runtime = lua_runtime_alloc();
    REQUIRE(runtime != nullptr);

    REQUIRE_EQ(lua_runtime_eval(runtime, "on_show = 'not callable'\n", "test"), ERROR_NONE);

    CHECK_EQ(lua_runtime_call(runtime, "on_show"), ERROR_INVALID_ARGUMENT);
    CHECK_FALSE(std::string(lua_runtime_get_error(runtime)).empty());

    // Missing entirely, rather than the wrong type
    CHECK_EQ(lua_runtime_call(runtime, "never_defined"), ERROR_INVALID_ARGUMENT);

    lua_runtime_free(runtime);
}

/** A failed call must leave nothing behind, or repeated lifecycle calls would overflow. */
TEST_CASE("repeated failing calls do not grow the Lua stack") {
    auto* runtime = lua_runtime_alloc();
    REQUIRE(runtime != nullptr);

    REQUIRE_EQ(lua_runtime_eval(runtime, "function boom() error('x') end\n", "test"), ERROR_NONE);

    for (int i = 0; i < 200; i++) {
        CHECK_NE(lua_runtime_call(runtime, "boom"), ERROR_NONE);
        CHECK_EQ(lua_runtime_call(runtime, "missing"), ERROR_INVALID_ARGUMENT);
    }

    // Still usable afterwards
    CHECK_EQ(lua_runtime_eval(runtime, "local x = 1\n", "test"), ERROR_NONE);

    lua_runtime_free(runtime);
}
