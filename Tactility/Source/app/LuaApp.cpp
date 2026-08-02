#include <Tactility/app/alertdialog/AlertDialog.h>
#include <Tactility/app/LuaApp.h>
#include <Tactility/app/App.h>
#include <Tactility/file/File.h>
#include <Tactility/Assets.h>

#include <lua/bindings.h>
#include <lua/runtime.h>
#include <luavgl/bindings.h>
#include <lvgl/lvgl.h>

#include <format>
#include <string>
#include <tactility/log.h>
#include <utility>

namespace tt::app {

constexpr auto* TAG = "LuaApp";

/**
 * An app whose behaviour is a Lua script.
 *
 * Unlike ElfApp there is no per-platform binary and no loader: the script is read at
 * onCreate and interpreted, so one .app runs on every target including the simulator.
 *
 * The script defines its lifecycle as global functions, all optional:
 *
 *     function on_create() end
 *     function on_show() end     -- lvgl.root() is the app's screen
 *     function on_hide() end
 *     function on_destroy() end
 *
 * The chunk itself runs once at onCreate, receiving the app's install directory as its
 * first vararg (`local app_dir = ...`), matching how ELF apps locate their assets.
 */
class LuaApp final : public App {

    const std::string appPath;
    const std::string appId;
    LuaRuntime* runtime = nullptr;
    /** Whether luavgl_bindings_open() has run, so teardown knows to detach. */
    bool bindingsOpen = false;

    /**
     * Reports a script failure and closes the app.
     *
     * Stopping by id rather than with the argument-less stop(), which stops whichever app
     * is on top: alertdialog::start() pushes the dialog above this one, so "top" is the
     * dialog by the time this runs. Without the id the dialog closes itself and leaves a
     * broken app behind it.
     *
     * The app is stopped rather than merely reported because a script that failed during
     * its lifecycle has no working UI - `on_show` raising part-way through leaves a
     * half-built screen the user cannot do anything with.
     */
    void failWith(const std::string& context) const {
        const char* message = lua_runtime_get_error(runtime);
        LOG_E(TAG, "%s: %s", context.c_str(), message);
        alertdialog::start("Script error", std::format("{}: {}", context, message));
        stop(appId);
    }

    /**
     * Calls an optional lifecycle function.
     *
     * A missing one is normal - a script that only draws needs no on_hide - so absence is
     * silent and only a raised error is reported.
     */
    void callOptional(const char* name, bool fatal = true) {
        if (runtime == nullptr || !lua_runtime_has_function(runtime, name)) {
            return;
        }

        if (lua_runtime_call(runtime, name) != ERROR_NONE) {
            if (fatal) {
                failWith(name);
            } else {
                // Teardown callbacks report but do not stop: the app is already on its way
                // out, and asking the loader to stop it again mid-teardown would re-enter
                // the path currently running.
                LOG_E(TAG, "%s: %s", name, lua_runtime_get_error(runtime));
            }
        }
    }

public:

    LuaApp(std::string appPath, std::string appId) :
        appPath(std::move(appPath)),
        appId(std::move(appId)) {}

    void onCreate(AppContext& appContext) override {
        const std::string script_path = std::format("{}/lua/main.lua", appPath);
        LOG_I(TAG, "Starting Lua app %s", script_path.c_str());

        runtime = lua_runtime_alloc();
        if (runtime == nullptr) {
            LOG_E(TAG, "Out of memory");
            alertdialog::start("Error", "Application failed to start: out of memory");
            stop(appId);
            return;
        }

        lua_bindings_open(runtime);

        // The chunk receives the install directory, so `app_dir .. "/assets/x.png"` works
        // the same from SD and from internal flash.
        if (lua_runtime_eval_file(runtime, script_path.c_str(), appPath.c_str()) != ERROR_NONE) {
            failWith("Failed to load script");
            return;
        }

        callOptional("on_create");
    }

    void onShow(AppContext& appContext, lv_obj_t* parent) override {
        if (runtime == nullptr) {
            return;
        }

        // Opened here rather than in onCreate because the bindings need the screen the app
        // draws into, and that only exists once it is being shown. GuiService already holds
        // the LVGL lock around this call, which is the contract luavgl expects.
        luavgl_bindings_open(runtime, parent);
        bindingsOpen = true;

        callOptional("on_show");
    }

    void onHide(AppContext& appContext) override {
        // Not fatal: hiding is usually the first step of closing, so stopping the app here
        // would be redundant at best and re-entrant at worst.
        callOptional("on_hide", false);
    }

    void onDestroy(AppContext& appContext) override {
        callOptional("on_destroy", false);

        if (runtime != nullptr) {
            // Order matters and is the whole teardown story. Widgets and timers the script
            // created routinely outlive this app - the loader tears the screen down after
            // onDestroy - and each carries a callback pointing into this lua_State.
            // Detaching them before closing the state is what stops the eventual
            // LV_EVENT_DELETE from calling into freed memory.
            //
            // The lock is taken here because, unlike onShow/onHide, onDestroy runs on the
            // loader thread with no LVGL lock held.
            if (bindingsOpen) {
                lvgl_lock();
                luavgl_bindings_close(runtime);
                lvgl_unlock();
                bindingsOpen = false;
            }

            lua_runtime_free(runtime);
            runtime = nullptr;
        }
    }
};

std::shared_ptr<App> createLuaApp(const std::shared_ptr<AppManifest>& manifest) {
    LOG_I(TAG, "createLuaApp");
    assert(manifest != nullptr);
    assert(manifest->appLocation.isExternal());
    return std::make_shared<LuaApp>(manifest->appLocation.getPath(), manifest->appId);
}

} // namespace
