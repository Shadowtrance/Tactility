#include "doctest.h"

#include <cstring>
#include <lua/bindings.h>
#include <lua/module.h>
#include <lua/runtime.h>

extern "C" {
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
}

namespace {

/** Looks up a symbol in the module's table, as the ELF loader's resolver would. */
const void* find_symbol(const char* name) {
    for (const auto* symbol = lua_module.symbols; symbol->name != nullptr; symbol++) {
        if (std::strcmp(symbol->name, name) == 0) {
            return symbol->symbol;
        }
    }
    return nullptr;
}

}

TEST_CASE("the vendored interpreter is the expected version") {
    // The SDK exports headers for apps to compile against and resolves the matching
    // symbols at load time, so firmware and apps must agree on the version. A silent
    // change here would break every already-built app.
    CHECK_EQ(LUA_VERSION_NUM, 505);
}

TEST_CASE("the module declares itself correctly") {
    CHECK_EQ(std::strcmp(lua_module.name, "lua"), 0);
    REQUIRE(lua_module.symbols != nullptr);
}

TEST_CASE("the module exports the runtime API to side-loaded apps") {
    // Walk to the terminator, collecting names, then check the API is fully represented.
    // A function added to the header but forgotten here would be invisible to ELF apps.
    bool found_alloc = false;
    bool found_free = false;
    bool found_eval = false;
    bool found_eval_file = false;
    bool found_get_error = false;
    bool found_get_memory_used = false;

    for (const auto* symbol = lua_module.symbols; symbol->name != nullptr; symbol++) {
        CHECK(symbol->symbol != nullptr);

        if (std::strcmp(symbol->name, "lua_runtime_alloc") == 0) found_alloc = true;
        if (std::strcmp(symbol->name, "lua_runtime_free") == 0) found_free = true;
        if (std::strcmp(symbol->name, "lua_runtime_eval") == 0) found_eval = true;
        if (std::strcmp(symbol->name, "lua_runtime_eval_file") == 0) found_eval_file = true;
        if (std::strcmp(symbol->name, "lua_runtime_get_error") == 0) found_get_error = true;
        if (std::strcmp(symbol->name, "lua_runtime_get_memory_used") == 0) found_get_memory_used = true;
    }

    CHECK(found_alloc);
    CHECK(found_free);
    CHECK(found_eval);
    CHECK(found_eval_file);
    CHECK(found_get_error);
    CHECK(found_get_memory_used);
}

TEST_CASE("an exported symbol is the real function") {
    const void* exported = find_symbol("lua_runtime_alloc");
    REQUIRE(exported != nullptr);
    CHECK_EQ(exported, (const void*)&lua_runtime_alloc);
}

TEST_CASE("the lua C API is exported for ELF apps") {
    // A side-loaded app that embeds Lua resolves these at load time rather than linking
    // its own interpreter. Spot-check across each header, including the pieces this
    // module's own bindings rely on, so a regenerated list that dropped a group fails here.
    const char* required[] = {
        // lua.h - state, stack, calls
        "lua_newstate", "lua_close", "lua_pcallk", "lua_callk", "lua_error",
        "lua_gettop", "lua_settop", "lua_pushvalue", "lua_rotate",
        "lua_pushnil", "lua_pushboolean", "lua_pushinteger", "lua_pushlstring",
        "lua_pushcclosure", "lua_pushfstring", "lua_toboolean", "lua_tolstring",
        "lua_tointegerx", "lua_touserdata", "lua_type", "lua_typename",
        "lua_createtable", "lua_getfield", "lua_setfield", "lua_getglobal",
        "lua_setglobal", "lua_geti", "lua_seti", "lua_rawlen", "lua_next",
        "lua_getmetatable", "lua_setmetatable", "lua_newuserdatauv", "lua_gc",
        // lauxlib.h. Note luaL_openlibs is absent on purpose: since 5.5 it is a macro
        // over luaL_openselectedlibs, so it has no address to export.
        "luaL_newstate", "luaL_openselectedlibs", "luaL_makeseed",
        "luaL_loadbufferx", "luaL_loadfilex",
        "luaL_checklstring", "luaL_checkinteger", "luaL_checkudata", "luaL_testudata",
        "luaL_newmetatable", "luaL_setmetatable", "luaL_setfuncs", "luaL_error",
        "luaL_traceback", "luaL_tolstring", "luaL_len", "luaL_ref", "luaL_unref",
        // lualib.h - so a script can open libraries selectively
        "luaopen_base", "luaopen_string", "luaopen_table", "luaopen_math",
        "luaopen_io", "luaopen_os", "luaopen_coroutine", "luaopen_utf8",
    };

    for (const auto* name : required) {
        INFO("symbol: ", name);
        CHECK(find_symbol(name) != nullptr);
    }
}

TEST_CASE("exported lua symbols point at the real implementations") {
    CHECK_EQ(find_symbol("lua_newstate"), (const void*)&lua_newstate);
    CHECK_EQ(find_symbol("luaL_openselectedlibs"), (const void*)&luaL_openselectedlibs);
    CHECK_EQ(find_symbol("lua_pcallk"), (const void*)&lua_pcallk);
    CHECK_EQ(find_symbol("luaL_checkudata"), (const void*)&luaL_checkudata);
}

TEST_CASE("the symbol table is large enough to be the full API, and well formed") {
    size_t count = 0;
    for (const auto* symbol = lua_module.symbols; symbol->name != nullptr; symbol++) {
        CHECK(symbol->symbol != nullptr);
        CHECK(std::strlen(symbol->name) > 0);
        count++;
    }

    // 156 Lua API entries plus this module's own; a regeneration that silently produced
    // a near-empty list would still pass the spot-checks above if they were unlucky.
    CHECK(count > 150);
}
