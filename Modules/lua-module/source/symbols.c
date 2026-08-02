// SPDX-License-Identifier: Apache-2.0
//
// The Lua C API, exported so that side-loaded ELF apps bind to the interpreter that is
// already in the firmware instead of linking their own copy.
//
// This list is the intersection of what the public headers declare and what the built
// archive actually defines. The block below the GENERATED BEGIN marker is written by
// generate-symbols.py - regenerate rather than editing it by hand:
//
//     ninja -C buildsim lua && python Modules/lua-module/generate-symbols.py
//
// lua-module's own API, above that marker, is curated and left alone by the script.
//
// The macro-only parts of the API are absent by nature, and there are more of them than
// you might expect: lua_pcall, lua_call, lua_tostring, luaL_loadbuffer and - since 5.5 -
// luaL_openlibs are all #defines over the functions below, so they resolve inside the
// app at compile time and need no symbol here.

#include <lua/bindings.h>
#include <lua/module.h>
#include <lua/runtime.h>

#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>

const struct ModuleSymbol lua_module_symbols[] = {
    // lua-module's own API: enough for an app to drive a runtime without
    // touching the raw Lua C API below.
    DEFINE_MODULE_SYMBOL(lua_runtime_alloc),
    DEFINE_MODULE_SYMBOL(lua_runtime_free),
    DEFINE_MODULE_SYMBOL(lua_runtime_eval),
    DEFINE_MODULE_SYMBOL(lua_runtime_eval_file),
    DEFINE_MODULE_SYMBOL(lua_runtime_get_error),
    DEFINE_MODULE_SYMBOL(lua_runtime_get_memory_used),
    DEFINE_MODULE_SYMBOL(lua_runtime_get_state),
    DEFINE_MODULE_SYMBOL(lua_bindings_open),

    // GENERATED BEGIN - see generate-symbols.py
    // lua core
    DEFINE_MODULE_SYMBOL(lua_absindex),
    DEFINE_MODULE_SYMBOL(lua_arith),
    DEFINE_MODULE_SYMBOL(lua_atpanic),
    DEFINE_MODULE_SYMBOL(lua_callk),
    DEFINE_MODULE_SYMBOL(lua_checkstack),
    DEFINE_MODULE_SYMBOL(lua_close),
    DEFINE_MODULE_SYMBOL(lua_closeslot),
    DEFINE_MODULE_SYMBOL(lua_closethread),
    DEFINE_MODULE_SYMBOL(lua_compare),
    DEFINE_MODULE_SYMBOL(lua_concat),
    DEFINE_MODULE_SYMBOL(lua_copy),
    DEFINE_MODULE_SYMBOL(lua_createtable),
    DEFINE_MODULE_SYMBOL(lua_dump),
    DEFINE_MODULE_SYMBOL(lua_error),
    DEFINE_MODULE_SYMBOL(lua_gc),
    DEFINE_MODULE_SYMBOL(lua_getallocf),
    DEFINE_MODULE_SYMBOL(lua_getfield),
    DEFINE_MODULE_SYMBOL(lua_getglobal),
    DEFINE_MODULE_SYMBOL(lua_gethook),
    DEFINE_MODULE_SYMBOL(lua_gethookcount),
    DEFINE_MODULE_SYMBOL(lua_gethookmask),
    DEFINE_MODULE_SYMBOL(lua_geti),
    DEFINE_MODULE_SYMBOL(lua_getinfo),
    DEFINE_MODULE_SYMBOL(lua_getiuservalue),
    DEFINE_MODULE_SYMBOL(lua_getlocal),
    DEFINE_MODULE_SYMBOL(lua_getmetatable),
    DEFINE_MODULE_SYMBOL(lua_getstack),
    DEFINE_MODULE_SYMBOL(lua_gettable),
    DEFINE_MODULE_SYMBOL(lua_gettop),
    DEFINE_MODULE_SYMBOL(lua_getupvalue),
    DEFINE_MODULE_SYMBOL(lua_iscfunction),
    DEFINE_MODULE_SYMBOL(lua_isinteger),
    DEFINE_MODULE_SYMBOL(lua_isnumber),
    DEFINE_MODULE_SYMBOL(lua_isstring),
    DEFINE_MODULE_SYMBOL(lua_isuserdata),
    DEFINE_MODULE_SYMBOL(lua_isyieldable),
    DEFINE_MODULE_SYMBOL(lua_len),
    DEFINE_MODULE_SYMBOL(lua_load),
    DEFINE_MODULE_SYMBOL(lua_newstate),
    DEFINE_MODULE_SYMBOL(lua_newthread),
    DEFINE_MODULE_SYMBOL(lua_newuserdatauv),
    DEFINE_MODULE_SYMBOL(lua_next),
    DEFINE_MODULE_SYMBOL(lua_numbertocstring),
    DEFINE_MODULE_SYMBOL(lua_pcallk),
    DEFINE_MODULE_SYMBOL(lua_pushboolean),
    DEFINE_MODULE_SYMBOL(lua_pushcclosure),
    DEFINE_MODULE_SYMBOL(lua_pushexternalstring),
    DEFINE_MODULE_SYMBOL(lua_pushfstring),
    DEFINE_MODULE_SYMBOL(lua_pushinteger),
    DEFINE_MODULE_SYMBOL(lua_pushlightuserdata),
    DEFINE_MODULE_SYMBOL(lua_pushlstring),
    DEFINE_MODULE_SYMBOL(lua_pushnil),
    DEFINE_MODULE_SYMBOL(lua_pushnumber),
    DEFINE_MODULE_SYMBOL(lua_pushstring),
    DEFINE_MODULE_SYMBOL(lua_pushthread),
    DEFINE_MODULE_SYMBOL(lua_pushvalue),
    DEFINE_MODULE_SYMBOL(lua_pushvfstring),
    DEFINE_MODULE_SYMBOL(lua_rawequal),
    DEFINE_MODULE_SYMBOL(lua_rawget),
    DEFINE_MODULE_SYMBOL(lua_rawgeti),
    DEFINE_MODULE_SYMBOL(lua_rawgetp),
    DEFINE_MODULE_SYMBOL(lua_rawlen),
    DEFINE_MODULE_SYMBOL(lua_rawset),
    DEFINE_MODULE_SYMBOL(lua_rawseti),
    DEFINE_MODULE_SYMBOL(lua_rawsetp),
    DEFINE_MODULE_SYMBOL(lua_resume),
    DEFINE_MODULE_SYMBOL(lua_rotate),
    DEFINE_MODULE_SYMBOL(lua_setallocf),
    DEFINE_MODULE_SYMBOL(lua_setfield),
    DEFINE_MODULE_SYMBOL(lua_setglobal),
    DEFINE_MODULE_SYMBOL(lua_sethook),
    DEFINE_MODULE_SYMBOL(lua_seti),
    DEFINE_MODULE_SYMBOL(lua_setiuservalue),
    DEFINE_MODULE_SYMBOL(lua_setlocal),
    DEFINE_MODULE_SYMBOL(lua_setmetatable),
    DEFINE_MODULE_SYMBOL(lua_settable),
    DEFINE_MODULE_SYMBOL(lua_settop),
    DEFINE_MODULE_SYMBOL(lua_setupvalue),
    DEFINE_MODULE_SYMBOL(lua_setwarnf),
    DEFINE_MODULE_SYMBOL(lua_status),
    DEFINE_MODULE_SYMBOL(lua_stringtonumber),
    DEFINE_MODULE_SYMBOL(lua_toboolean),
    DEFINE_MODULE_SYMBOL(lua_tocfunction),
    DEFINE_MODULE_SYMBOL(lua_toclose),
    DEFINE_MODULE_SYMBOL(lua_tointegerx),
    DEFINE_MODULE_SYMBOL(lua_tolstring),
    DEFINE_MODULE_SYMBOL(lua_tonumberx),
    DEFINE_MODULE_SYMBOL(lua_topointer),
    DEFINE_MODULE_SYMBOL(lua_tothread),
    DEFINE_MODULE_SYMBOL(lua_touserdata),
    DEFINE_MODULE_SYMBOL(lua_type),
    DEFINE_MODULE_SYMBOL(lua_typename),
    DEFINE_MODULE_SYMBOL(lua_upvalueid),
    DEFINE_MODULE_SYMBOL(lua_upvaluejoin),
    DEFINE_MODULE_SYMBOL(lua_version),
    DEFINE_MODULE_SYMBOL(lua_warning),
    DEFINE_MODULE_SYMBOL(lua_xmove),
    DEFINE_MODULE_SYMBOL(lua_yieldk),

    // auxiliary library
    DEFINE_MODULE_SYMBOL(luaL_addgsub),
    DEFINE_MODULE_SYMBOL(luaL_addlstring),
    DEFINE_MODULE_SYMBOL(luaL_addstring),
    DEFINE_MODULE_SYMBOL(luaL_addvalue),
    DEFINE_MODULE_SYMBOL(luaL_alloc),
    DEFINE_MODULE_SYMBOL(luaL_argerror),
    DEFINE_MODULE_SYMBOL(luaL_buffinit),
    DEFINE_MODULE_SYMBOL(luaL_buffinitsize),
    DEFINE_MODULE_SYMBOL(luaL_callmeta),
    DEFINE_MODULE_SYMBOL(luaL_checkany),
    DEFINE_MODULE_SYMBOL(luaL_checkinteger),
    DEFINE_MODULE_SYMBOL(luaL_checklstring),
    DEFINE_MODULE_SYMBOL(luaL_checknumber),
    DEFINE_MODULE_SYMBOL(luaL_checkoption),
    DEFINE_MODULE_SYMBOL(luaL_checkstack),
    DEFINE_MODULE_SYMBOL(luaL_checktype),
    DEFINE_MODULE_SYMBOL(luaL_checkudata),
    DEFINE_MODULE_SYMBOL(luaL_checkversion_),
    DEFINE_MODULE_SYMBOL(luaL_error),
    DEFINE_MODULE_SYMBOL(luaL_execresult),
    DEFINE_MODULE_SYMBOL(luaL_fileresult),
    DEFINE_MODULE_SYMBOL(luaL_getmetafield),
    DEFINE_MODULE_SYMBOL(luaL_getsubtable),
    DEFINE_MODULE_SYMBOL(luaL_gsub),
    DEFINE_MODULE_SYMBOL(luaL_len),
    DEFINE_MODULE_SYMBOL(luaL_loadbufferx),
    DEFINE_MODULE_SYMBOL(luaL_loadfilex),
    DEFINE_MODULE_SYMBOL(luaL_loadstring),
    DEFINE_MODULE_SYMBOL(luaL_makeseed),
    DEFINE_MODULE_SYMBOL(luaL_newmetatable),
    DEFINE_MODULE_SYMBOL(luaL_newstate),
    DEFINE_MODULE_SYMBOL(luaL_openselectedlibs),
    DEFINE_MODULE_SYMBOL(luaL_optinteger),
    DEFINE_MODULE_SYMBOL(luaL_optlstring),
    DEFINE_MODULE_SYMBOL(luaL_optnumber),
    DEFINE_MODULE_SYMBOL(luaL_prepbuffsize),
    DEFINE_MODULE_SYMBOL(luaL_pushresult),
    DEFINE_MODULE_SYMBOL(luaL_pushresultsize),
    DEFINE_MODULE_SYMBOL(luaL_ref),
    DEFINE_MODULE_SYMBOL(luaL_requiref),
    DEFINE_MODULE_SYMBOL(luaL_setfuncs),
    DEFINE_MODULE_SYMBOL(luaL_setmetatable),
    DEFINE_MODULE_SYMBOL(luaL_testudata),
    DEFINE_MODULE_SYMBOL(luaL_tolstring),
    DEFINE_MODULE_SYMBOL(luaL_traceback),
    DEFINE_MODULE_SYMBOL(luaL_typeerror),
    DEFINE_MODULE_SYMBOL(luaL_unref),
    DEFINE_MODULE_SYMBOL(luaL_where),

    // standard libraries
    DEFINE_MODULE_SYMBOL(luaopen_base),
    DEFINE_MODULE_SYMBOL(luaopen_coroutine),
    DEFINE_MODULE_SYMBOL(luaopen_debug),
    DEFINE_MODULE_SYMBOL(luaopen_io),
    DEFINE_MODULE_SYMBOL(luaopen_math),
    DEFINE_MODULE_SYMBOL(luaopen_os),
    DEFINE_MODULE_SYMBOL(luaopen_package),
    DEFINE_MODULE_SYMBOL(luaopen_string),
    DEFINE_MODULE_SYMBOL(luaopen_table),
    DEFINE_MODULE_SYMBOL(luaopen_utf8),
    // GENERATED END

    MODULE_SYMBOL_TERMINATOR
};
