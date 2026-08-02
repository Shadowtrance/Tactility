// SPDX-License-Identifier: Apache-2.0
#include <lua/module.h>

extern "C" {

/** Defined in symbols.c: this module's own API plus the whole Lua C API. */
extern const struct ModuleSymbol lua_module_symbols[];

// No start/stop: the interpreter holds no global state of its own, so there is nothing
// to bring up. Each runtime is independent and created on demand.
Module lua_module = {
    .name = "lua",
    .start = nullptr,
    .stop = nullptr,
    .drivers = nullptr,
    .symbols = lua_module_symbols,
    .internal = nullptr
};

}
