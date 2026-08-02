#pragma once

#include "AppManifest.h"

namespace tt::app {

/**
 * Creates an app that runs a Lua script.
 *
 * Unlike createElfApp() this has no ESP_PLATFORM guard: a Lua app needs an interpreter,
 * not a loader, so the same app runs on every target including the simulator.
 */
std::shared_ptr<App> createLuaApp(const std::shared_ptr<AppManifest>& manifest);

}
