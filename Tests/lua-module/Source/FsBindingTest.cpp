#include "doctest.h"

#include <cstdio>
#include <lua/bindings.h>
#include <lua/runtime.h>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

namespace {

struct BoundRuntime {
    LuaRuntime* runtime = lua_runtime_alloc();

    BoundRuntime() { lua_bindings_open(runtime); }
    ~BoundRuntime() { lua_runtime_free(runtime); }

    error_t eval(const char* code) const { return lua_runtime_eval(runtime, code, "test"); }
    std::string error() const { return lua_runtime_get_error(runtime); }
};

/** A directory that exists for the duration of the scope, emptied on the way out. */
struct ScopedDirectory {
    std::string path;

    explicit ScopedDirectory(const char* name) : path(name) {
        remove_all();
        REQUIRE(::mkdir(path.c_str(), 0777) == 0);
    }

    ~ScopedDirectory() { remove_all(); }

    void remove_all() const {
        // Only ever one level deep in these tests
        for (const auto* child : { "a.txt", "b.txt", "nested" }) {
            const auto full = path + "/" + child;
            ::unlink(full.c_str());
            ::rmdir(full.c_str());
        }
        ::rmdir(path.c_str());
    }
};

std::string read_file(const std::string& path) {
    std::string content;
    if (auto* file = std::fopen(path.c_str(), "rb")) {
        char buffer[512];
        size_t read = 0;
        while ((read = std::fread(buffer, 1, sizeof(buffer), file)) > 0) {
            content.append(buffer, read);
        }
        std::fclose(file);
    }
    return content;
}

}

TEST_CASE("the fs table is present") {
    const BoundRuntime lua;
    CHECK_EQ(lua.eval("assert(type(tactility.fs) == 'table')"), ERROR_NONE);
}

// Reading and writing -----------------------------------------------------------------

TEST_CASE("write then read returns the same content") {
    const ScopedDirectory dir("lua_fs_test");
    const BoundRuntime lua;

    const auto* code =
        "assert(tactility.fs.write('lua_fs_test/a.txt', 'hello world'))\n"
        "assert(tactility.fs.read('lua_fs_test/a.txt') == 'hello world')";
    CHECK_EQ(lua.eval(code), ERROR_NONE);

    CHECK_EQ(read_file("lua_fs_test/a.txt"), "hello world");
}

TEST_CASE("write truncates and append extends") {
    const ScopedDirectory dir("lua_fs_test");
    const BoundRuntime lua;

    const auto* code =
        "tactility.fs.write('lua_fs_test/a.txt', 'first')\n"
        "tactility.fs.write('lua_fs_test/a.txt', 'second')\n"
        "assert(tactility.fs.read('lua_fs_test/a.txt') == 'second')\n"
        "tactility.fs.append('lua_fs_test/a.txt', '-third')\n"
        "assert(tactility.fs.read('lua_fs_test/a.txt') == 'second-third')";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("content with embedded nulls and newlines survives a round trip") {
    const ScopedDirectory dir("lua_fs_test");
    const BoundRuntime lua;

    // Lua strings are length-counted, so a null byte must not truncate the write
    const auto* code =
        "local original = 'a\\0b\\nc\\r\\nd'\n"
        "assert(tactility.fs.write('lua_fs_test/a.txt', original))\n"
        "local loaded = tactility.fs.read('lua_fs_test/a.txt')\n"
        "assert(loaded == original, ('got %d bytes, expected %d'):format(#loaded, #original))";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("an empty file reads back as an empty string, not nil") {
    const ScopedDirectory dir("lua_fs_test");
    const BoundRuntime lua;

    const auto* code =
        "assert(tactility.fs.write('lua_fs_test/a.txt', ''))\n"
        "local loaded = tactility.fs.read('lua_fs_test/a.txt')\n"
        "assert(loaded == '', type(loaded))";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("a larger file round trips intact") {
    const ScopedDirectory dir("lua_fs_test");
    const BoundRuntime lua;

    // Bigger than the read path's internal buffering, to catch a partial read
    const auto* code =
        "local original = string.rep('0123456789', 5000)\n"
        "assert(tactility.fs.write('lua_fs_test/a.txt', original))\n"
        "assert(tactility.fs.size('lua_fs_test/a.txt') == 50000)\n"
        "assert(tactility.fs.read('lua_fs_test/a.txt') == original)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("reading a missing file returns nil and a reason") {
    const BoundRuntime lua;

    const auto* code =
        "local content, reason = tactility.fs.read('/definitely/not/here.txt')\n"
        "assert(content == nil)\n"
        "assert(type(reason) == 'string' and #reason > 0, tostring(reason))";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("writing to an unwritable path returns nil and a reason") {
    const BoundRuntime lua;

    const auto* code =
        "local ok, reason = tactility.fs.write('/definitely/not/here/a.txt', 'x')\n"
        "assert(ok == nil)\n"
        "assert(type(reason) == 'string' and #reason > 0, tostring(reason))";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

// Inspection --------------------------------------------------------------------------

TEST_CASE("exists, is_file and is_directory agree with reality") {
    const ScopedDirectory dir("lua_fs_test");
    const BoundRuntime lua;

    const auto* code =
        "tactility.fs.write('lua_fs_test/a.txt', 'x')\n"
        "assert(tactility.fs.exists('lua_fs_test/a.txt'))\n"
        "assert(tactility.fs.is_file('lua_fs_test/a.txt'))\n"
        "assert(not tactility.fs.is_directory('lua_fs_test/a.txt'))\n"
        "assert(tactility.fs.exists('lua_fs_test'))\n"
        "assert(tactility.fs.is_directory('lua_fs_test'))\n"
        "assert(not tactility.fs.is_file('lua_fs_test'))\n"
        "assert(not tactility.fs.exists('lua_fs_test/nope.txt'))\n"
        "assert(not tactility.fs.is_file('lua_fs_test/nope.txt'))\n"
        "assert(not tactility.fs.is_directory('lua_fs_test/nope.txt'))";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("size reports bytes, and fails on a missing file") {
    const ScopedDirectory dir("lua_fs_test");
    const BoundRuntime lua;

    const auto* code =
        "tactility.fs.write('lua_fs_test/a.txt', 'twelve bytes')\n"
        "assert(tactility.fs.size('lua_fs_test/a.txt') == 12)\n"
        "local size, reason = tactility.fs.size('lua_fs_test/nope.txt')\n"
        "assert(size == nil and type(reason) == 'string')";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

// Directories -------------------------------------------------------------------------

TEST_CASE("list returns entries without the dot directories") {
    const ScopedDirectory dir("lua_fs_test");
    const BoundRuntime lua;

    const auto* code =
        "tactility.fs.write('lua_fs_test/a.txt', 'x')\n"
        "tactility.fs.write('lua_fs_test/b.txt', 'y')\n"
        "assert(tactility.fs.make_directory('lua_fs_test/nested'))\n"
        "local entries = tactility.fs.list('lua_fs_test')\n"
        "assert(#entries == 3, 'expected 3, got ' .. #entries)\n"
        "local names = {}\n"
        "for _, e in ipairs(entries) do\n"
        "  assert(e.name ~= '.' and e.name ~= '..', 'dot entry leaked: ' .. e.name)\n"
        "  names[e.name] = e\n"
        "end\n"
        "assert(names['a.txt'] and names['b.txt'] and names['nested'])\n"
        "assert(names['nested'].directory == true)\n"
        "assert(names['a.txt'].directory == false)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("listing an empty directory returns an empty table") {
    const ScopedDirectory dir("lua_fs_test");
    const BoundRuntime lua;

    const auto* code =
        "local entries = tactility.fs.list('lua_fs_test')\n"
        "assert(type(entries) == 'table')\n"
        "assert(#entries == 0)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("listing a missing directory returns nil and a reason") {
    const BoundRuntime lua;

    const auto* code =
        "local entries, reason = tactility.fs.list('/definitely/not/here')\n"
        "assert(entries == nil)\n"
        "assert(type(reason) == 'string' and #reason > 0)";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("make_directory is idempotent") {
    const ScopedDirectory dir("lua_fs_test");
    const BoundRuntime lua;

    // A second call must not fail: EEXIST is success for this API
    const auto* code =
        "assert(tactility.fs.make_directory('lua_fs_test/nested'))\n"
        "assert(tactility.fs.make_directory('lua_fs_test/nested'))\n"
        "assert(tactility.fs.is_directory('lua_fs_test/nested'))";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("remove deletes a file and an empty directory") {
    const ScopedDirectory dir("lua_fs_test");
    const BoundRuntime lua;

    const auto* code =
        "tactility.fs.write('lua_fs_test/a.txt', 'x')\n"
        "assert(tactility.fs.remove('lua_fs_test/a.txt'))\n"
        "assert(not tactility.fs.exists('lua_fs_test/a.txt'))\n"
        "tactility.fs.make_directory('lua_fs_test/nested')\n"
        "assert(tactility.fs.remove('lua_fs_test/nested'))\n"
        "assert(not tactility.fs.exists('lua_fs_test/nested'))";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("remove fails on a missing path and on a non-empty directory") {
    const ScopedDirectory dir("lua_fs_test");
    const BoundRuntime lua;

    const auto* code =
        "local ok, reason = tactility.fs.remove('lua_fs_test/nope.txt')\n"
        "assert(ok == nil and type(reason) == 'string')\n"
        "tactility.fs.write('lua_fs_test/a.txt', 'x')\n"
        "local ok2, reason2 = tactility.fs.remove('lua_fs_test')\n"
        "assert(ok2 == nil and type(reason2) == 'string')";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

// Mounts ------------------------------------------------------------------------------

TEST_CASE("mounts returns a table") {
    const BoundRuntime lua;

    // The simulator may have no mounted filesystems, so only the shape is guaranteed
    const auto* code =
        "local mounts = tactility.fs.mounts()\n"
        "assert(type(mounts) == 'table')\n"
        "for _, path in ipairs(mounts) do assert(type(path) == 'string') end";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

TEST_CASE("user_data_path returns a string or a reason") {
    const BoundRuntime lua;

    const auto* code =
        "local path, reason = tactility.fs.user_data_path()\n"
        "assert((type(path) == 'string' and #path > 0) or (path == nil and type(reason) == 'string'))";
    CHECK_EQ(lua.eval(code), ERROR_NONE);
}

// Argument handling -------------------------------------------------------------------

TEST_CASE("a missing argument raises rather than crashing") {
    const BoundRuntime lua;
    CHECK_NE(lua.eval("tactility.fs.read()"), ERROR_NONE);
    CHECK_NE(lua.eval("tactility.fs.write('lua_fs_test/a.txt')"), ERROR_NONE);
    CHECK_NE(lua.eval("tactility.fs.list()"), ERROR_NONE);
}

TEST_CASE("a non-string path raises rather than being coerced silently") {
    const BoundRuntime lua;
    CHECK_NE(lua.eval("tactility.fs.read({})"), ERROR_NONE);
    CHECK_NE(lua.eval("tactility.fs.exists(true)"), ERROR_NONE);
}

TEST_CASE("repeated reads do not leak lua memory") {
    const ScopedDirectory dir("lua_fs_test");
    const BoundRuntime lua;

    CHECK_EQ(lua.eval("tactility.fs.write('lua_fs_test/a.txt', string.rep('x', 4096))"), ERROR_NONE);
    CHECK_EQ(lua.eval("collectgarbage('collect')"), ERROR_NONE);
    const size_t baseline = lua_runtime_get_memory_used(lua.runtime);

    const auto* code =
        "for i = 1, 100 do\n"
        "  local content = tactility.fs.read('lua_fs_test/a.txt')\n"
        "  assert(#content == 4096)\n"
        "end\n"
        "collectgarbage('collect')";
    CHECK_EQ(lua.eval(code), ERROR_NONE);

    // 100 reads of 4 KB each: a leak of even one buffer per call would be obvious
    CHECK(lua_runtime_get_memory_used(lua.runtime) < baseline + 32768);
}
