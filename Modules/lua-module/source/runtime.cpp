// SPDX-License-Identifier: Apache-2.0
#include <lua/runtime.h>
#include <lua/runtime_private.h>

#include <tactility/log.h>
#include <tactility/memory.h>

#include <cstdio>
#include <cstring>
#include <new>

extern "C" {
#include <lauxlib.h>
#include <lualib.h>
}

constexpr auto* TAG = "lua";

/**
 * Lua's heap is a large number of small, short-lived allocations, so it is the ideal
 * tenant for PSRAM where one exists: it keeps script churn away from the internal heap
 * that drivers and DMA need. This is only 'desired' - devices without PSRAM fall back to
 * internal memory and simply have a smaller ceiling.
 */
constexpr MemoryPolicy LUA_MEMORY_POLICY = {
    .required = 0,
    .desired = MEMORY_CAPABILITY_EXTERNAL,
    .alignment = 0
};

/**
 * Lua's allocator contract: `ptr` is the block (NULL when allocating fresh), `original_size`
 * its previous size, and `new_size` the requested one, where 0 means free.
 */
static void* allocate(void* context, void* ptr, size_t original_size, size_t new_size) {
    auto* runtime = static_cast<LuaRuntime*>(context);

    if (new_size == 0) {
        memory_free(ptr);
        runtime->memory_used -= (ptr != nullptr) ? original_size : 0;
        return nullptr;
    }

    auto* result = memory_realloc_with_policy(ptr, new_size, &LUA_MEMORY_POLICY);
    if (result == nullptr) {
        // Returning NULL makes Lua raise a memory error rather than abort, so the failure
        // surfaces as a catchable script error. The old block is still valid and still ours.
        return nullptr;
    }

    runtime->memory_used += new_size - ((ptr != nullptr) ? original_size : 0);
    return result;
}

/** Records a message from the top of the Lua stack, then pops it. */
static void store_error_from_stack(LuaRuntime* runtime) {
    const char* message = lua_tostring(runtime->state, -1);
    if (message == nullptr) {
        message = "unknown error";
    }

    std::strncpy(runtime->error_message, message, ERROR_MESSAGE_CAPACITY - 1);
    runtime->error_message[ERROR_MESSAGE_CAPACITY - 1] = '\0';

    lua_pop(runtime->state, 1);
}

/**
 * Runs as the message handler for lua_pcall, while the erroring frames are still on the
 * stack, so the traceback it attaches points at the failing line rather than at the call
 * site. Without this a script error reports only the message.
 */
static int add_traceback(lua_State* state) {
    const char* message = lua_tostring(state, 1);
    if (message == nullptr) {
        message = "(non-string error)";
    }

    luaL_traceback(state, state, message, 1);
    return 1;
}

/** Calls a function already on the stack, with `argument_count` args below it. */
static error_t protected_call(LuaRuntime* runtime, int argument_count) {
    auto* state = runtime->state;

    // The handler is pushed below the function so its stack index stays fixed.
    const int handler_index = lua_gettop(state) - argument_count;
    lua_pushcfunction(state, add_traceback);
    lua_insert(state, handler_index);

    const int result = lua_pcall(state, argument_count, 0, handler_index);

    lua_remove(state, handler_index);

    if (result != LUA_OK) {
        store_error_from_stack(runtime);
        return (result == LUA_ERRMEM) ? ERROR_OUT_OF_MEMORY : ERROR_UNDEFINED;
    }

    return ERROR_NONE;
}

extern "C" {

struct LuaRuntime* lua_runtime_alloc(void) {
    auto* runtime = new (std::nothrow) LuaRuntime();
    if (runtime == nullptr) {
        LOG_E(TAG, "Out of memory allocating runtime");
        return nullptr;
    }

    // The third argument seeds string hashing, which is what keeps a script from being
    // able to force hash collisions in its own tables. luaL_makeseed() is what
    // luaL_newstate() itself passes; we only call lua_newstate() directly to install the
    // allocator above.
    runtime->state = lua_newstate(allocate, runtime, luaL_makeseed(nullptr));
    if (runtime->state == nullptr) {
        LOG_E(TAG, "Out of memory allocating interpreter");
        delete runtime;
        return nullptr;
    }

    // Opens the full standard library. Deliberate: ELF apps already run with full access,
    // so a Lua-only sandbox would not match the platform's trust model. 5.5's
    // luaL_openselectedlibs() is there if a narrower runtime is ever wanted.
    luaL_openlibs(runtime->state);

    return runtime;
}

void lua_runtime_free(struct LuaRuntime* runtime) {
    // Closing runs every pending finalizer and frees the heap through allocate() above.
    lua_close(runtime->state);
    delete runtime;
}

error_t lua_runtime_eval(struct LuaRuntime* runtime, const char* code, const char* chunk_name) {
    runtime->error_message[0] = '\0';

    // The '=' prefix tells Lua to use the name verbatim in messages, instead of quoting
    // it as a source string.
    char name_buffer[64];
    if (chunk_name != nullptr) {
        snprintf(name_buffer, sizeof(name_buffer), "=%s", chunk_name);
    } else {
        std::strcpy(name_buffer, "=(load)");
    }

    if (luaL_loadbuffer(runtime->state, code, std::strlen(code), name_buffer) != LUA_OK) {
        store_error_from_stack(runtime);
        return ERROR_INVALID_ARGUMENT; // A compile error is bad input, not a failed run.
    }

    return protected_call(runtime, 0);
}

error_t lua_runtime_eval_file(struct LuaRuntime* runtime, const char* path, const char* arg) {
    runtime->error_message[0] = '\0';

    if (luaL_loadfile(runtime->state, path) != LUA_OK) {
        store_error_from_stack(runtime);
        // luaL_loadfile reports both "cannot open" and syntax errors this way. The
        // message distinguishes them; the caller mainly needs to know it did not run.
        return ERROR_INVALID_ARGUMENT;
    }

    int argument_count = 0;
    if (arg != nullptr) {
        lua_pushstring(runtime->state, arg);
        argument_count = 1;
    }

    return protected_call(runtime, argument_count);
}

bool lua_runtime_has_function(const struct LuaRuntime* runtime, const char* name) {
    lua_getglobal(runtime->state, name);
    const bool found = lua_isfunction(runtime->state, -1);
    lua_pop(runtime->state, 1);
    return found;
}

error_t lua_runtime_call(struct LuaRuntime* runtime, const char* name) {
    runtime->error_message[0] = '\0';

    lua_getglobal(runtime->state, name);

    if (!lua_isfunction(runtime->state, -1)) {
        lua_pop(runtime->state, 1);
        snprintf(runtime->error_message, sizeof(runtime->error_message),
                 "'%s' is not a function", name);
        return ERROR_INVALID_ARGUMENT;
    }

    return protected_call(runtime, 0);
}

const char* lua_runtime_get_error(const struct LuaRuntime* runtime) {
    return runtime->error_message;
}

size_t lua_runtime_get_memory_used(const struct LuaRuntime* runtime) {
    return runtime->memory_used;
}

void* lua_runtime_get_state(const struct LuaRuntime* runtime) {
    return runtime->state;
}

}
