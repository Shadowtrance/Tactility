// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <tactility/error.h>

#ifdef __cplusplus
extern "C" {
#endif

struct LuaRuntime;
struct _lv_obj_t;

/**
 * @brief Open the LVGL bindings in a runtime.
 *
 * Adds a global `lvgl` table. Separate from lua_bindings_open() so a runtime can have
 * kernel access without a UI, and so this module stays optional.
 *
 * @warning The caller must hold the LVGL lock for the lifetime of every script that uses
 * these bindings, and the script must run on a single thread. Nothing here takes the lock
 * itself: a script makes many small LVGL calls, and locking per call would both thrash and
 * leave gaps mid-sequence where another task could see a half-built UI.
 *
 * @param runtime the runtime
 * @param root the object new widgets parent themselves to when none is given, normally
 *             the app's screen. May be NULL, in which case the active screen is used.
 * @return ERROR_NONE on success
 */
error_t luavgl_bindings_open(struct LuaRuntime* runtime, struct _lv_obj_t* root);

/**
 * @brief Detach the bindings from a runtime, before closing it.
 *
 * **Call this before lua_runtime_free() on any runtime the bindings were opened in.**
 * Widgets routinely outlive the runtime that created them - an app closes its Lua state
 * while its LVGL tree is still on screen - and each wrapped widget holds a delete callback
 * pointing at that state. Without this, tearing the tree down afterwards calls into a
 * freed lua_State.
 *
 * Safe to call on a runtime the bindings were never opened in.
 *
 * @param runtime the runtime
 */
void luavgl_bindings_close(struct LuaRuntime* runtime);

#ifdef __cplusplus
}
#endif
