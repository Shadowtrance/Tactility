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

/** Clears the namespaces these tests use, so a previous run cannot affect this one. */
struct CleanNamespaces {
    CleanNamespaces() { clear(); }
    ~CleanNamespaces() { clear(); }

    static void clear() {
        auto* runtime = lua_runtime_alloc();
        lua_bindings_open(runtime);
        lua_runtime_eval(
            runtime,
            "for _, ns in ipairs({'luatest', 'luatest2'}) do tactility.settings.clear(ns) end",
            "cleanup"
        );
        lua_runtime_free(runtime);
    }
};

}

TEST_CASE("the settings table is present") {
    const BoundRuntime lua;
    CHECK_EQ(lua.eval("assert(type(tactility.settings) == 'table')"), ERROR_NONE);
}

TEST_CASE("set then get returns the value") {
    const CleanNamespaces clean;
    const BoundRuntime lua;

    const auto* code =
        "assert(tactility.settings.set('luatest', 'name', 'tactility'))\n"
        "assert(tactility.settings.get('luatest', 'name') == 'tactility')";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("values persist across runtimes") {
    const CleanNamespaces clean;

    {
        const BoundRuntime first;
        CHECK_EQ(first.eval("tactility.settings.set('luatest', 'persisted', 'yes')"), ERROR_NONE);
    }

    // A fresh runtime, so this only passes if the value actually reached storage
    const BoundRuntime second;
    CHECK_EQ(second.eval("assert(tactility.settings.get('luatest', 'persisted') == 'yes')"), ERROR_NONE);
}

TEST_CASE("get returns the default when a key is absent") {
    const CleanNamespaces clean;
    const BoundRuntime lua;

    const auto* code =
        "assert(tactility.settings.get('luatest', 'missing') == nil)\n"
        "assert(tactility.settings.get('luatest', 'missing', 'fallback') == 'fallback')\n"
        "assert(tonumber(tactility.settings.get('luatest', 'count', '0')) == 0)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("set replaces rather than duplicating") {
    const CleanNamespaces clean;
    const BoundRuntime lua;

    const auto* code =
        "tactility.settings.set('luatest', 'k', 'first')\n"
        "tactility.settings.set('luatest', 'k', 'second')\n"
        "assert(tactility.settings.get('luatest', 'k') == 'second')\n"
        "local keys = tactility.settings.keys('luatest')\n"
        "assert(#keys == 1, 'expected 1 key, got ' .. #keys)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("namespaces are isolated from each other") {
    const CleanNamespaces clean;
    const BoundRuntime lua;

    const auto* code =
        "tactility.settings.set('luatest', 'k', 'one')\n"
        "tactility.settings.set('luatest2', 'k', 'two')\n"
        "assert(tactility.settings.get('luatest', 'k') == 'one')\n"
        "assert(tactility.settings.get('luatest2', 'k') == 'two')";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("has reports presence") {
    const CleanNamespaces clean;
    const BoundRuntime lua;

    const auto* code =
        "assert(tactility.settings.has('luatest', 'k') == false)\n"
        "tactility.settings.set('luatest', 'k', 'v')\n"
        "assert(tactility.settings.has('luatest', 'k') == true)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("remove deletes a key and reports whether it existed") {
    const CleanNamespaces clean;
    const BoundRuntime lua;

    const auto* code =
        "tactility.settings.set('luatest', 'k', 'v')\n"
        "assert(tactility.settings.remove('luatest', 'k') == true)\n"
        "assert(tactility.settings.has('luatest', 'k') == false)\n"
        "assert(tactility.settings.remove('luatest', 'k') == false)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("keys lists everything stored") {
    const CleanNamespaces clean;
    const BoundRuntime lua;

    const auto* code =
        "tactility.settings.set('luatest', 'alpha', '1')\n"
        "tactility.settings.set('luatest', 'beta', '2')\n"
        "tactility.settings.set('luatest', 'gamma', '3')\n"
        "local found = {}\n"
        "for _, k in ipairs(tactility.settings.keys('luatest')) do found[k] = true end\n"
        "assert(found.alpha and found.beta and found.gamma)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("keys on an empty namespace returns an empty table") {
    const CleanNamespaces clean;
    const BoundRuntime lua;

    const auto* code =
        "local keys = tactility.settings.keys('luatest')\n"
        "assert(type(keys) == 'table' and #keys == 0)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("clear empties a namespace") {
    const CleanNamespaces clean;
    const BoundRuntime lua;

    const auto* code =
        "tactility.settings.set('luatest', 'a', '1')\n"
        "tactility.settings.set('luatest', 'b', '2')\n"
        "assert(tactility.settings.clear('luatest'))\n"
        "assert(#tactility.settings.keys('luatest') == 0)\n"
        "assert(tactility.settings.get('luatest', 'a') == nil)\n"
        "assert(tactility.settings.clear('luatest'))"; // already empty is still success
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

// Value fidelity ------------------------------------------------------------------------

TEST_CASE("values containing newlines survive a round trip") {
    const CleanNamespaces clean;
    const BoundRuntime lua;

    // The store is line-based, so an unescaped newline would split one value into two
    // entries - or silently truncate it.
    const auto* code =
        "local original = 'line one\\nline two\\r\\nline three'\n"
        "tactility.settings.set('luatest', 'multiline', original)\n"
        "local loaded = tactility.settings.get('luatest', 'multiline')\n"
        "assert(loaded == original, ('got %q'):format(tostring(loaded)))\n"
        "assert(#tactility.settings.keys('luatest') == 1)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("values containing the separator survive a round trip") {
    const CleanNamespaces clean;
    const BoundRuntime lua;

    // "=" is the key/value separator; only the first one may be treated as such
    const auto* code =
        "local original = 'a=b=c'\n"
        "tactility.settings.set('luatest', 'equals', original)\n"
        "assert(tactility.settings.get('luatest', 'equals') == original)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("values containing backslashes survive a round trip") {
    const CleanNamespaces clean;
    const BoundRuntime lua;

    // The escape character itself must be escaped, or "\\n" becomes a newline on load
    const auto* code =
        "local original = 'C:\\\\path\\\\to\\\\file and a literal \\\\n'\n"
        "tactility.settings.set('luatest', 'backslash', original)\n"
        "assert(tactility.settings.get('luatest', 'backslash') == original)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("an empty value round trips and is distinct from absent") {
    const CleanNamespaces clean;
    const BoundRuntime lua;

    const auto* code =
        "tactility.settings.set('luatest', 'empty', '')\n"
        "assert(tactility.settings.get('luatest', 'empty') == '')\n"
        "assert(tactility.settings.has('luatest', 'empty') == true)\n"
        "assert(tactility.settings.get('luatest', 'never_set') == nil)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("a long value round trips") {
    const CleanNamespaces clean;
    const BoundRuntime lua;

    const auto* code =
        "local original = string.rep('abcdefghij', 400)\n"
        "tactility.settings.set('luatest', 'long', original)\n"
        "assert(tactility.settings.get('luatest', 'long') == original)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

// Validation ----------------------------------------------------------------------------

TEST_CASE("path traversal in a namespace or key is rejected") {
    const BoundRuntime lua;

    // These become part of a file path, so they must not be able to escape the directory
    CHECK_NE(lua.eval("tactility.settings.set('../escape', 'k', 'v')"), ERROR_NONE);
    CHECK_NE(lua.eval("tactility.settings.set('a/b', 'k', 'v')"), ERROR_NONE);
    CHECK_NE(lua.eval("tactility.settings.get('luatest', '../escape')"), ERROR_NONE);
    CHECK_NE(lua.eval("tactility.settings.get('luatest', 'a/b')"), ERROR_NONE);
}

TEST_CASE("an empty or hidden namespace is rejected") {
    const BoundRuntime lua;
    CHECK_NE(lua.eval("tactility.settings.set('', 'k', 'v')"), ERROR_NONE);
    CHECK_NE(lua.eval("tactility.settings.set('.hidden', 'k', 'v')"), ERROR_NONE);
    CHECK_NE(lua.eval("tactility.settings.get('luatest', '')"), ERROR_NONE);
}

TEST_CASE("missing or wrongly typed arguments raise") {
    const BoundRuntime lua;
    CHECK_NE(lua.eval("tactility.settings.get()"), ERROR_NONE);
    CHECK_NE(lua.eval("tactility.settings.get('luatest')"), ERROR_NONE);
    CHECK_NE(lua.eval("tactility.settings.set('luatest', 'k')"), ERROR_NONE);
    CHECK_NE(lua.eval("tactility.settings.set('luatest', 'k', {})"), ERROR_NONE);
    CHECK_NE(lua.eval("tactility.settings.keys()"), ERROR_NONE);
}

TEST_CASE("repeated writes do not leak lua memory") {
    const CleanNamespaces clean;
    const BoundRuntime lua;

    CHECK_EQ(lua.eval("collectgarbage('collect')"), ERROR_NONE);
    const size_t baseline = lua_runtime_get_memory_used(lua.runtime);

    const auto* code =
        "for i = 1, 50 do\n"
        "  tactility.settings.set('luatest', 'k' .. (i % 5), tostring(i))\n"
        "  assert(tactility.settings.get('luatest', 'k' .. (i % 5)) == tostring(i))\n"
        "end\n"
        "collectgarbage('collect')";
    CHECK_EQ(lua.eval(code), ERROR_NONE);

    CHECK(lua_runtime_get_memory_used(lua.runtime) < baseline + 32768);
}
