// SPDX-License-Identifier: Apache-2.0
#include <luavgl/bindings.h>
#include <luavgl/module.h>

extern "C" {

static const ModuleSymbol luavgl_module_symbols[] = {
    DEFINE_MODULE_SYMBOL(luavgl_bindings_open),
    DEFINE_MODULE_SYMBOL(luavgl_bindings_close),
    MODULE_SYMBOL_TERMINATOR
};

// No start/stop: the bindings hold no state of their own. Everything lives in the Lua
// runtime they are opened into, and LVGL is brought up by lvgl-module.
Module luavgl_module = {
    .name = "luavgl",
    .start = nullptr,
    .stop = nullptr,
    .drivers = nullptr,
    .symbols = luavgl_module_symbols,
    .internal = nullptr
};

}
