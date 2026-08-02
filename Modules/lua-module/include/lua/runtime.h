// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <tactility/error.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * An isolated Lua interpreter.
 *
 * One runtime holds one lua_State plus the buffer for the last error. Runtimes do not
 * share globals, so an app cannot reach another app's state. A runtime is not
 * thread-safe: use it from a single thread, or guard it externally.
 *
 * The type is opaque so that <lua.h> stays out of the public headers, which keeps this
 * usable from C++ and from code that has no include path to the interpreter.
 */
struct LuaRuntime;

/**
 * @brief Create a Lua runtime with the standard library loaded.
 * @return a new runtime, or NULL on allocation failure
 */
struct LuaRuntime* lua_runtime_alloc(void);

/**
 * @brief Destroy a runtime and everything allocated inside it.
 * @param runtime the runtime
 */
void lua_runtime_free(struct LuaRuntime* runtime);

/**
 * @brief Compile and run a string of Lua source.
 * @param runtime the runtime
 * @param code the source to run
 * @param chunk_name name used for this chunk in error messages and tracebacks, or NULL
 *                   to label it "=(load)"
 * @return ERROR_NONE on success, otherwise the error is retrievable via
 *         lua_runtime_get_error()
 */
error_t lua_runtime_eval(struct LuaRuntime* runtime, const char* code, const char* chunk_name);

/**
 * @brief Compile and run a Lua source file.
 * @param runtime the runtime
 * @param path absolute path to the script
 * @param arg an extra string passed to the chunk as its first vararg (`local x = ...`),
 *            or NULL to pass nothing. Apps receive their install directory this way.
 * @return ERROR_NONE on success, otherwise the error is retrievable via
 *         lua_runtime_get_error()
 */
error_t lua_runtime_eval_file(struct LuaRuntime* runtime, const char* path, const char* arg);

/**
 * @brief Whether the runtime has a global function by this name.
 *
 * Lets a caller skip an optional callback without the cost (and the error handling) of
 * calling something that is not there.
 *
 * @param runtime the runtime
 * @param name the global's name
 * @return true if the global exists and is callable
 */
bool lua_runtime_has_function(const struct LuaRuntime* runtime, const char* name);

/**
 * @brief Call a global function that takes no arguments and returns nothing.
 *
 * Used for app lifecycle callbacks. Calling a name that does not exist is an error, so
 * check with lua_runtime_has_function() first for optional ones.
 *
 * @param runtime the runtime
 * @param name the global's name
 * @return ERROR_NONE on success, otherwise the message is retrievable via
 *         lua_runtime_get_error()
 */
error_t lua_runtime_call(struct LuaRuntime* runtime, const char* name);

/**
 * @brief Get the message from the most recent failure on this runtime.
 * @param runtime the runtime
 * @return the message, or an empty string if nothing has failed yet. Owned by the
 *         runtime and overwritten by the next failure.
 */
const char* lua_runtime_get_error(const struct LuaRuntime* runtime);

/**
 * @brief Get the number of bytes currently allocated by this runtime.
 * @param runtime the runtime
 * @return bytes in use by the Lua heap
 */
size_t lua_runtime_get_memory_used(const struct LuaRuntime* runtime);

/**
 * @brief Get the underlying lua_State.
 *
 * For code that adds its own bindings to a runtime - `luavgl-module` uses this to install
 * the LVGL table. The return type is void* so that callers who do not need <lua.h> are
 * not forced to include it; cast it to lua_State* where you do.
 *
 * @param runtime the runtime
 * @return the lua_State, never NULL for a successfully allocated runtime
 */
void* lua_runtime_get_state(const struct LuaRuntime* runtime);

#ifdef __cplusplus
}
#endif
