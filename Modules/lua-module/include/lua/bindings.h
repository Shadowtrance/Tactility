// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <tactility/error.h>

#ifdef __cplusplus
extern "C" {
#endif

struct LuaRuntime;

/**
 * @brief Open the Tactility bindings in a runtime.
 *
 * Adds a global `tactility` table holding the bound API, grouped into sub-tables by
 * subsystem (`tactility.log`, `tactility.device`, ...). Safe to call more than once;
 * the second call replaces the table.
 *
 * The runtime is usable without this - it is separate from lua_runtime_alloc() so that
 * a caller can run a plain script with no access to the device at all.
 *
 * @param runtime the runtime
 * @return ERROR_NONE on success
 */
error_t lua_bindings_open(struct LuaRuntime* runtime);

#ifdef __cplusplus
}
#endif
