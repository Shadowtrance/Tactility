# lua-module

Embeds a [Lua](https://www.lua.org/) 5.5 interpreter as a kernel module, so that scripts
can run on device without being compiled into the firmware.

A `.lua` file can be a complete app: UI via `luavgl-module`, packaged and installed like
any other. See `PLAN.md` for what is still outstanding.

## Status

Working:

- `LuaRuntime`, an isolated interpreter with the standard library loaded
- Evaluating source from a string or a file, with an argument passed as `...`
- Errors reported with a stack traceback instead of a bare message
- Allocation routed through the kernel allocator, preferring PSRAM, with byte accounting
- A `tactility` table with `log`, `device`, `fs`, `settings`, `time`, `power` and `wifi`
  bindings
- Calling script-defined global functions from C, for app lifecycle callbacks
- Builds for ESP32 and the POSIX simulator; 120 tests under `Tests/lua-module`, clean
  under AddressSanitizer

Not done:

- No GPIO, audio, display, bluetooth, lora, i2c, uart, ... - the aim is parity with what
  ELF apps can reach through the SDK, deferred until after the LVGL work so there is a
  real app to test against; see `PLAN.md`
- No LVGL bindings, so no UI - see the note below
- No app runtime: nothing registers a Lua app with the loader yet

`luaL_openlibs` opens the whole standard library. That is deliberate: ELF apps already
run with full access and no sandbox, and Lua apps are meant to reach as much of the
device as they do, so a Lua-only sandbox would not match the platform's trust model.
The practical limits are narrower than the policy: `os.execute` and `io.popen` are gone
because ESP32 has no process model, and `package.loadlib` cannot work without `dlopen`.
If a narrower runtime is ever wanted for a specific app, 5.5's `luaL_openselectedlibs`
takes a bitmask of which libraries to open.

## Cost

The interpreter, standard library and bindings add **~141 KB** of flash (measured on
esp32p4: 0x33d980 to 0x361b80; the bindings are ~2 KB of that, and 5.5 costs ~9 KB more
than 5.4 for the incremental GC and the new array representation).

**Every device pays this today.** `Tactility.cpp` calls `module_ensure_started(&lua_module)`
unconditionally, which is what publishes the symbols - and starting the module is also what
stops the linker discarding it. There is no way to have the symbols without the flash.
Making this opt-in is checkpoint 4 in `PLAN.md`.

A fresh runtime with the bindings opened holds **~16.7 KB** of heap on device (measured on
m5stack-tab5), against ~19 KB on the simulator - `LUA_32BITS` halving every number payload
accounts for the difference. It lands in PSRAM where available, keeping script churn off
the internal heap that drivers and DMA need.

## Bindings

```lua
tactility.log.info("hello")                 -- also error/warning/debug/verbose
tactility.log.info("my-tag", "hello")       -- with an explicit tag

for _, name in ipairs(tactility.device.list()) do
    tactility.log.info(name)
end

local device = tactility.device.find("i2c0")
if device then
    tactility.log.info(device:name(), device:type(), device:is_ready())
    device:release()
end
```

Lookups return `nil` plus a reason string rather than raising, since asking whether a
device exists is ordinary. `find_by_compatible(str)` matches on the driver's compatible
string instead of the name.

A handle holds a kernel reference (`device_get`). `release()` hands it back, and so does
the garbage collector if the script forgets - including when a runtime is closed with
handles still live, which is the case that matters when an app exits badly. Calling
`release()` twice is harmless; using a handle afterwards raises.

`list()` and `children()` return names, not handles, on purpose: referencing every device
at once would pin them all for as long as the script held the list.

### tactility.fs

```lua
local path = tactility.fs.user_data_path() .. "/notes.txt"

tactility.fs.write(path, "hello")          -- also append(path, text)
local text = tactility.fs.read(path)       -- nil, reason on failure

for _, entry in ipairs(tactility.fs.list(tactility.fs.user_data_path())) do
    tactility.log.info(entry.name, entry.directory)
end

tactility.fs.exists(path)                  -- also is_file / is_directory / size
tactility.fs.make_directory(path)          -- idempotent
tactility.fs.remove(path)                  -- file, or empty directory
tactility.fs.mounts()                      -- mounted roots, e.g. {"/sdcard"}
```

Failures return `nil` plus a message rather than raising, matching `device.find`.

**Use this rather than Lua's `io`.** Every call takes the mount's file mutex for its
duration; on boards where the SD card shares a bus with the display, that mutex is what
keeps a file read from interleaving with a display transfer. `io.open` bypasses it
entirely.

Reads and writes are whole-file. Holding a `FILE*` across a yield would hold the mutex
with it and stall the display, so there is no streaming API - if one is ever needed it
wants a handle type with explicit lifetime rules, like the device handles above.

### tactility.settings

```lua
tactility.settings.set("myapp", "username", "teron")
local name = tactility.settings.get("myapp", "username")
local count = tonumber(tactility.settings.get("myapp", "count", "0"))  -- with a default

tactility.settings.has("myapp", "username")
tactility.settings.remove("myapp", "username")
tactility.settings.keys("myapp")     -- every key in the namespace
tactility.settings.clear("myapp")    -- drop the namespace
```

Namespaced key/value storage that survives an OS upgrade, backed by one file per namespace
under the service user-data directory. Everything is a string; use `tonumber()` for
numbers.

Values are escaped, so newlines, `=` and backslashes survive a round trip. Writes go to a
temporary file and rename, so an interrupted write leaves the previous settings intact
rather than a truncated file.

Namespaces and keys accept letters, digits, `-`, `_` and `.`, and must not start with `.`.
Anything else raises rather than being sanitised: quietly rewriting a name would let two
different keys collide, and these become part of a file path.

### tactility.time

```lua
local start = tactility.time.uptime()        -- ms since boot, monotonic
tactility.time.sleep(50)                     -- blocks this thread
local elapsed = tactility.time.uptime() - start

local seconds = tactility.time.now()         -- unix time, or nil + "clock not set"
local t = tactility.time.date()              -- same shape as os.date("*t")
tactility.log.info(("%04d-%02d-%02d"):format(t.year, t.month, t.day))
```

`now()` returns `nil` plus a reason before NTP or an RTC has set the clock, rather than
reporting a 1970 date as if it were real. `date()` takes an optional timestamp and matches
`os.date("*t")` exactly, including 1-based months and weekdays.

`uptime_micros()` is a float rather than an integer: on ESP32 `LUA_32BITS` caps integers at
2^31, which microseconds overflow after about 36 minutes of uptime.

**`sleep` blocks the calling thread.** Once Lua apps drive the UI that will be the LVGL
task, and sleeping there freezes the display. It is for short waits, not for pacing an
app - timers are the right tool for that and are not bound yet.

### tactility.power

```lua
if tactility.power.is_available() then
    local pct = tactility.power.capacity()        -- 0-100
    local charging = tactility.power.is_charging()
    tactility.log.info(("battery %d%%%s"):format(pct, charging and " (charging)" or ""))
end

tactility.power.voltage()   -- mV
tactility.power.current()   -- mA, positive while charging
```

Not every board reports every property, so each returns `nil` plus a reason when
unsupported rather than a plausible-looking zero. `capabilities()` returns a table of
booleans (`capacity`, `voltage`, `current`, `is_charging`, `charge_control`,
`quick_charge`, `power_off`) so a UI can hide controls it cannot drive.

Charging can be controlled where the board supports it, via `is_allowed_to_charge()` and
`set_allowed_to_charge(bool)`. **`power_off` is deliberately not exposed** - a script
should not be able to take the device down outright.

### tactility.wifi

```lua
tactility.log.info(tactility.wifi.station_state())   -- "connected", "connecting", ...
tactility.log.info(tactility.wifi.ip_address())      -- nil + reason when disconnected
tactility.log.info(tactility.wifi.ssid(), tactility.wifi.rssi())

tactility.wifi.scan()                                -- starts, returns immediately
-- ... later, once is_scanning() is false ...
for _, ap in ipairs(tactility.wifi.scan_results()) do
    tactility.log.info(ap.ssid, ap.rssi, ap.channel, ap.authentication, ap.secured)
end
```

States are strings rather than numbers, so comparisons read properly. `scan()` is
asynchronous: a scan takes seconds and blocking would stall the calling task, so poll
`is_scanning()` and then read `scan_results()`.

**Connecting is not bound yet.** It takes a password, so exposing it means first deciding
how a script obtains credentials and whether it may join arbitrary networks unprompted.

### On reference counting and device_stop

`device.h` documents `device_stop()` as returning `ERROR_RESOURCE_BUSY` while references
are outstanding, but the implementation does not check the count - only `device_destruct()`
does. The tests here assert against destruct for that reason. Worth reconciling upstream:
either the doc or the check is wrong.

## Usage

```c
#include <lua/runtime.h>

struct LuaRuntime* runtime = lua_runtime_alloc();

if (lua_runtime_eval(runtime, "print('hello')", "example") != ERROR_NONE) {
    LOG_E(TAG, "%s", lua_runtime_get_error(runtime));
}

lua_runtime_free(runtime);
```

A runtime is single-threaded: use it from one thread, or guard it yourself.

## Numbers differ between targets

ESP32 builds use 32-bit integers and floats (`LUA_32BITS`), the simulator uses the
default 64-bit. Scripts doing large-integer or high-precision arithmetic can therefore
behave differently on device than in the simulator. The reasoning is in
`Libraries/lua/README.md`.

## Using this from an external app

Both halves of the SDK path are wired up:

- **Build time** - `release-sdk.py` exports `Libraries/lua` (public headers plus
  `liblua.a`) and `Modules/lua-module` (headers plus `liblua-module.a`), and the SDK's
  top-level `CMakeLists.txt` puts both on the include path and links them as `INTERFACE`.
  An app just needs `REQUIRES TactilitySDK`, as it already does.
- **Load time** - `source/symbols.c` exports this module's API plus the whole Lua C API
  (156 symbols) through the module symbol table, which is where the ELF loader's resolver
  ends up via `tt_symbol_resolver`. A side-loaded app therefore binds to the interpreter
  already in the firmware rather than linking its own copy.

The load-time half only works when the firmware has **started** the module:
`module_resolve_symbol_global()` walks started modules only. `Tactility.cpp` does this
alongside the other modules; without it an app fails at load with
`Can't find common lua_pcallk` even though the archive is linked in.

## Lua apps

An external app can be a script rather than an ELF binary. Its manifest says so:

```properties
app.runtime=lua
```

and the package holds `lua/main.lua` instead of `elf/<platform>.elf`. `AppInstance` then
builds a `LuaApp` (`Tactility/Source/app/LuaApp.cpp`) rather than an `ElfApp`.

Consequences, all in the good direction:

- **No per-platform build.** One `.app` runs on every device whose firmware has this
  module, so `target.platforms` means "known to work on these" rather than "compiled for
  these".
- **No ESP-IDF, and no network, to build one.** Packaging is "tar up three things".
- **It runs on the simulator.** `createLuaApp` sits outside the `#ifdef ESP_PLATFORM` that
  restricts `createElfApp`, because a script needs an interpreter rather than a loader.

The script defines its lifecycle as optional globals - `on_create`, `on_show`, `on_hide`,
`on_destroy` - and receives its install directory as the chunk's first vararg. See
`Apps/LuaHello` in the apps repo for a worked example.

### Script errors

A script that raises during `on_create` or `on_show` gets an alert dialog with the message
and traceback, and **the app is stopped** - a half-built screen is not something the user
can do anything with.

The app is stopped **by id**, not with the argument-less `stop()`, which stops whichever
app is on top. `alertdialog::start()` pushes the dialog above this one, so by the time the
stop runs, "top" is the dialog: without the id the dialog closes itself and leaves the
broken app behind it.

`on_hide` and `on_destroy` errors are logged but do not stop the app. Both run while it is
already on its way out, so asking the loader to stop it again would re-enter the path
currently running.

### Teardown ordering

`LuaApp::onDestroy` closes the LVGL bindings before freeing the runtime, and takes the
LVGL lock to do it. Both matter:

- Widgets and timers routinely **outlive the app** - the loader tears the screen down
  after `onDestroy` - and each carries a callback into the `lua_State`. Freeing the state
  first leaves those callbacks pointing at freed memory.
- `onShow`/`onHide` are called with the LVGL lock already held, but `onCreate`/`onDestroy`
  run on the loader thread without it, so `onDestroy` must take it itself.

## Where this is going

See `PLAN.md` for the full roadmap and its checkpoints. What is left is mostly breadth -
the remaining kernel bindings (`gpio`, `audio`, `i2c`, `uart`, ...) and developer
experience.
