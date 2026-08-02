# Lua apps for Tactility - plan

Working notes for adding Lua as a second app runtime alongside ELF. Written to be picked
up cold: each checkpoint says what to do, how to know it worked, and what it risks.

Status legend: `[x]` done, `[ ]` not started, `[~]` partially done.

---

## Where things stand

Done so far (checkpoints 1-4), **verified running on hardware** (m5stack-tab5):

- `Libraries/lua` - Lua 5.5.0 vendored, builds for ESP32 and the simulator
- `Modules/lua-module` - runtime; `log`, `device`, `fs`, `settings`, `time`, `power` and
  `wifi` bindings; the Lua C API exported as module symbols
- `Modules/luavgl-module` - LVGL bindings: widgets, the `set{}` dispatcher, widget
  lifetime coupling, the real Tactility toolbar, widget events, timers and the core
  widget set
- SDK export - headers, archives, and the symbol table, verified end to end
- 213 tests across both modules, clean under AddressSanitizer

`Apps/LuaSdkTest` loads and runs on device, carrying no interpreter of its own:

```
runtime eval: ok            <- lua-module's API, resolved from firmware symbols
device binding: ok          <- a script called into the kernel
runtime heap: 16705 bytes
raw C API: 10 (expect 10)   <- luaL_newstate/openlibs/getglobal resolved individually
lua: Lua 5.5.0

I (18061) luasdktest: devices: 35     <- Lua enumerating real hardware, logging via tactility.log
I (18052) memory: External: 29284012 / 33554432 available
```

The interpreter lands in PSRAM as intended, so script churn stays off the internal heap
that drivers and DMA need.

Cost: **~141 KB** flash on esp32p4, **~126 KB** on ESP32, **~16.7 KB** heap per runtime
(allocated in PSRAM). Every device carries it - decided deliberately in checkpoint 4, where
even the tightest 4MB board keeps 36% of its app partition free.

**A Lua script now draws a complete app on hardware** - toolbar, close button, and rows
showing battery, wifi, devices, mounts, settings, clock and its own heap. See
`Apps/LuaSdkTest` - which now also has a live-updating timer, a working button, the wider
widget set, and shared styles driving its row and control layout.

Checkpoint 6 is complete, and checkpoint 7 is written and building: `app.runtime=lua`,
`createLuaApp`, lifecycle callbacks, and `tactility.py` packaging. **A Lua app no longer
needs a C host at all** - `Apps/LuaHello` is one `.lua` file plus a manifest, packaged
without ESP-IDF, the network, or a per-platform build.

**Checkpoint 7 is verified on hardware.** `LuaHello.app` installs, launches, closes and
relaunches with working timers and touch.

Since then: key events (`lvgl.EVENT.KEY` handlers receive the key; `lvgl.KEY` constants)
and text-area editing (`cursor`/`set_cursor`/`insert`/`delete_char`), which existed to
support `Apps/LuaEditor` - a file browser and Lua editor, itself written in Lua. **Not yet
verified on hardware.**

---

## Checkpoint 1 - Lua builds on both targets `[x]`

Vendored **Lua 5.5.0** (released 22 Dec 2025), split into `Include/`/`Source/`, one
CMakeLists covering ESP-IDF and POSIX.

Two local changes in `luaconf.h`, bracketed `TACTILITY BEGIN/END`:
- `LUA_USE_POSIX` off on ESP32 - the feature macros are there but `popen`/`system` are stubs
- `LUA_32BITS` on ESP32 - no 64-bit FPU, so 32-bit numbers are faster and half the size

**Known consequence:** simulator and device arithmetic differ for large integers, and
bytecode is not portable between them. Source scripts are unaffected. If this ever bites,
the fix is to drop `LUA_32BITS` and accept the size/speed cost.

**Why 5.5 over 5.4:** it is the current stable release, and the migration cost was three
lines. 5.5 brings incremental *major* GC (5.4 does major collections in one stop-the-world
pass, which shows up as a hitch in an LVGL frame), `luaL_openselectedlibs` for opening a
subset of the standard library, `table.create`, and more compact arrays. The API deltas
that mattered: `lua_newstate` gained a string-hash seed argument, `lua_resetthread` and
`lua_setcstacklimit` went away, and `luaL_openlibs` became a macro over
`luaL_openselectedlibs` (so it is no longer an exportable symbol). `lauxlib.h` and
`lualib.h` were otherwise unchanged.

**Doing this later would have been worse:** apps resolve `lua_*` from firmware at load
time, and Lua does not promise binary compatibility across versions, so a Lua bump is an
SDK-version bump. Cheapest at zero shipped apps.

**Not using `espressif/lua`** (which is also 5.5.0). It is upstream Lua as a submodule
plus a `port/include/luaconf.h` that overrides `LUA_32BITS` and wires `LUAI_MAXSTACK` to
Kconfig - the same conclusions reached here. It is ESP-IDF-only, and the simulator needs
the same interpreter from the same source, so vendoring stays. Their header-injection
trick is worth remembering if this ever needs to stop editing `luaconf.h` in place.

---

## Checkpoint 2 - the module and its runtime `[x]`

`LuaRuntime`: an isolated interpreter, allocation routed through the kernel allocator
preferring PSRAM, errors carrying a stack traceback, byte accounting.

`lua_bindings_open()` is deliberately separate from `lua_runtime_alloc()`, so a runtime
with no device access at all remains possible.

---

## Checkpoint 3 - kernel bindings, SDK export, symbols `[x]`

`tactility.log` (five levels, optional tag) and `tactility.device` (list, find,
find_by_compatible, plus handle methods).

Device handles own a kernel reference and release it from `__gc`, so a script that forgets
- or a runtime that closes with handles live - does not strand references.

The Lua C API (156 symbols) is exported from `symbols.c` so a side-loaded ELF app binds to
the firmware's interpreter instead of linking its own.

**A module must be started for its symbols to exist.** Exporting a symbol table is not
enough: `module_resolve_symbol_global()` iterates *started* modules, so `Tactility.cpp`
needs a `module_ensure_started(&lua_module)` next to the others. Without it a side-loaded
app fails at load with:

```
E ELF: Can't find common lua_pcallk
E ElfApp: Application failed to load: missing symbol
```

which is easy to misread as a packaging problem. The same call is also what keeps the
linker from discarding the module - before it was added, adding `lua-module` to
`Tactility/CMakeLists.txt` left the firmware binary byte-identical, which looked like a
pleasant "costs nothing" result and was actually the symptom.

**This recurred verbatim with `luavgl-module`** despite being written down here, so
`release-sdk.py` now checks it: `module_is_started()` greps `Tactility.cpp` for the
matching `module_ensure_started(&<name>)`, and the export fails with an explanation rather
than shipping an SDK whose symbols cannot resolve. Adding a module to the SDK now means
adding all three:

1. `REQUIRES` in `Tactility/CMakeLists.txt` - so it builds
2. `module_ensure_started()` in `Tactility.cpp` - so its symbols resolve
3. the link list in `Tests/Tactility/CMakeLists.txt` - so the test binary still links

**The same error also appears when a symbol is missing rather than unstarted**, and there
are two separate causes. Both link fine and fail only at load, because undefined symbols
are normal for an ELF app.

1. *Declared and implemented, but not in the module's `symbols.c`.* Fix by using an
   exported equivalent, or adding the symbol to the table.
2. *Declared in the SDK's headers but not present in the firmware at all.* This was
   possible while the SDK's LVGL headers and the firmware's LVGL differed - fixed now
   (see below), but the same trap returns if the two ever drift apart again.

To tell them apart:

```bash
grep "DEFINE_MODULE_SYMBOL(the_function)" Modules/*/source/symbols.c   # exported?
grep -rn "the_function" managed_components/lvgl__lvgl/src/             # in the firmware's LVGL?
```

### FIXED: the SDK used to ship LVGL 9.4 headers against 9.3 firmware

`release-sdk.py` exports `Libraries/lvgl/src/**/*.h` - the simulator's tree - while the
firmware linked `lvgl/lvgl: "9.3.0"`. Apps compiled against 9.4 declarations and ran on
9.3, so anything added in 9.4 failed at load. It was **392 functions**, including whole
widgets (`lv_arclabel_*` did not exist in 9.3 at all).

Found via `lv_label_set_text_vfmt`, which is new in 9.4: in 9.3 the functionality existed
only as the internal `lv_text_set_text_vfmt`, never a public label API, so `lvgl-module`
could not have exported it.

**Resolved by bumping the firmware to `lvgl/lvgl: "9.4.0"`** (approved by bytewelder), so
both sides are 9.4.0 and the declared API sets are identical - 2436 functions each, gap of
zero. Costs ~14 KB of flash. `esp_lvgl_port 2.7.2` declares `lvgl >=8,<10`, so it was never
the constraint.

To check the two are still in step after a future bump:

```bash
grep -h LVGL_VERSION_MINOR Libraries/lvgl/lv_version.h managed_components/lvgl__lvgl/lv_version.h
```

The `nm -u` check in checkpoint 8 is still worth doing - it catches the *other* cause,
where a function exists in both but is missing from a module's `symbols.c`.

**Finding worth carrying forward:** `device.h` documents `device_stop()` as failing with
`ERROR_RESOURCE_BUSY` while references are held. It does not - only `device_destruct()`
checks. Either the doc or the check is wrong; worth raising upstream.

**On building apps:** apps are not built with a bare `idf.py build`. They go through
`tactility.py` in the TactilityApps repo:

```bash
python tactility.py Apps/<Name> build --local-sdk --verbose
```

That script sets `TACTILITY_SDK_PATH`, copies the right `sdkconfig`, runs the build, and
packages the result. Crucially it **ignores the build's exit code** and checks whether
`.app.elf` appeared instead - the elf cmake script always reports failure, and the
"undefined reference" lines for firmware-resident symbols (`lvgl_toolbar_create`,
`setElfAppParameters`, and now the `lua_*` symbols) are expected. Those resolve at load
time via `tt_symbol_resolver`. Running `idf.py build` directly on an app looks broken and
is not.

`--local-sdk` reads `TACTILITY_SDK_PATH` as a *parent* directory and appends
`{version}-{platform}/TactilitySDK`, which is why the local SDK lives in
`TactilityApps/TactilitySDKLocal/`.

`TactilityApps/Apps/LuaSdkTest` is a throwaway app written to verify this checkpoint: it
drives Lua both through `lua-module`'s runtime API and through the raw Lua C API, carrying
no interpreter of its own. Delete it once checkpoint 6 gives us a real example app.

---

## Checkpoint 4 - decide how Lua ships `[x]`

**Decided: always on, no opt-in.** Measured rather than guessed - the constrained case has
plenty of room.

`cyd-2432s028r` (ESP32, 4MB, `partitions-4mb-with-sd.csv`, the tightest partition at
`3800k` against `4M` everywhere else), built both ways:

| | Binary | App partition free |
| --- | --- | --- |
| Without Lua | 0x243b60 | 39% (1.45 MB) |
| With Lua | 0x263140 | 36% (1.35 MB) |

**~126 KB on ESP32** - cheaper than the 141 KB on esp32p4, and about 8.5% of the free
space that was there anyway. Fleet is 28x 16MB, 11x 8MB, 12x 4MB; the 4MB tier is all
CYD/Elecrow/Guition boards and they absorb it comfortably.

Kconfig opt-in and a separate firmware variant were both rejected: they add build-matrix
complexity and CI time to solve a problem the measurements say does not exist. If
something later eats the remaining 1.35 MB, the cheap fallback is to restrict Lua to 8MB+
devices - one condition, no new build permutations.

<details>
<summary>Original analysis, kept for the reasoning</summary>

Every device now carries ~141 KB of Lua, whether or not it will ever run a Lua app.
That is fine for development and wrong for release.

This is not optional-by-accident: `Tactility.cpp` must call
`module_ensure_started(&lua_module)` for the symbols to resolve at load time, and that
same call is what stops the linker discarding the interpreter. Symbols and flash come
together - any opt-in has to gate both.

Options, in order of preference:

1. **Kconfig opt-in** (`CONFIG_TT_LUA_ENABLED`, wired through `device.py` from
   `device.properties`). Devices choose. The SDK export becomes per-target, which the
   export script already handles for chip-restricted drivers.
2. **Always on.** Simplest, and the status quo. 141 KB is ~3.5% of a 4MB app partition -
   probably acceptable, but it is a tax on devices that will never run a Lua app.
3. **Separate firmware variant.** Most flexible, most release-engineering work.

**Do this before checkpoint 7**, since the app runtime is what makes the cost real.

</details>

The one part of the original analysis that still matters: **symbols and flash cannot be
separated.** `Tactility.cpp` must call `module_ensure_started(&lua_module)` for symbols to
resolve at load time, and that same call is what stops the linker discarding the
interpreter. Any future opt-in has to gate both together.

---

## Checkpoint 5 - more kernel bindings `[~]` (demo-useful set done)

**Goal: parity with ELF apps.** If an ELF app can reach it through the SDK, a Lua script
should be able to reach it too, unless something genuinely prevents it. The surface to
match is 386 symbols in `TactilityKernel/source/symbols.c` plus the `tt_*` headers in
`TactilityC/Include/`:

```
bluetooth 37   device 34   audio 25   display 18   gpio 17   wifi 16
thread 16      service 16  lora 16    i2c 15       uart 12   driver 11
timer 10       power 10    pointer 10 module 10    pwm 9     file 9
i2s 8          camera 8    usb 6      memory 5     backlight 5 ...
```

The one structural obstacle found so far: **anything implemented in the Tactility C++
layer is out of reach.** `lua-module` is a kernel module and can only require
`TactilityKernel`, so `Preferences`, `File.h` and the app framework cannot be called
directly - they have to be reimplemented against kernel primitives (as `fs` and `settings`
do) or reached another way. `gps-module` hits the same wall and solves it the same way.

The pattern is set: one file per subsystem in `source/bindings/`, one
`lua_bindings_open_*` each, registered in `bindings.cpp`.

Rough order, easiest and most useful first:

- [x] `tactility.fs` - read/write/append/exists/is_file/is_directory/size/list/
      make_directory/remove/mounts/user_data_path. **26 tests**, clean under ASan, ~1.9 KB
      of flash.

      Higher value than it looked. The kernel's `file_system.h` is mount management only -
      the actual file I/O in `Tactility/file/File.h` is C++ in a layer `lua-module` cannot
      depend on. So this binds POSIX I/O directly, but wraps every call in the mount's
      **file mutex** (`file_mutex_get`/`lock`/`unlock`). That is the real reason the
      binding has to exist: on boards where the SD card shares a bus with the display,
      that mutex is what stops a file read interleaving with a display transfer, and
      Lua's own `io.open` knows nothing about it.

      Whole-file read/write only, deliberately - a script holding a `FILE*` across a yield
      would hold the mount's mutex with it and stall the display. Streaming would need a
      handle type with its own lifetime rules; add only if something needs it.
- [x] `tactility.settings` - get/set/remove/has/keys/clear, namespaced. **17 tests.**

      File-backed, one `.properties` file per namespace under the service user-data
      directory, for the same reason as `fs`: the `Preferences` API that ELF apps reach
      via `tt_preferences.h` is C++ in the Tactility layer, which a kernel module cannot
      depend on. `gps-module` solves it the same way. Values survive an OS upgrade.

      Values are escaped, so newlines, `=` and backslashes round trip - a naive properties
      file corrupts all three. Writes go to a temporary and rename, so an interrupted
      write leaves the previous settings intact. Namespaces and keys are validated rather
      than sanitised, since they become part of a path and silently rewriting one would
      make two keys collide.

- [x] `tactility.time` - uptime/uptime_micros/now/date/sleep. **11 tests.**

      `date()` matches `os.date('*t')`'s shape and 1-based month/wday, checked against it
      directly in the tests. `now()` returns nil plus "clock not set" before NTP or an RTC
      has run, rather than reporting a 1970 date as real. `uptime_micros` is a float
      because `LUA_32BITS` would overflow at ~36 minutes of uptime.
- [x] `tactility.power` - is_available/capabilities/capacity/voltage/current/is_charging/
      is_allowed_to_charge/set_allowed_to_charge. **10 tests.**

      `capabilities()` reports what the board's driver actually supports, so a UI can hide
      controls it cannot drive. Unsupported properties return nil plus a reason rather
      than a plausible-looking zero. `power_off()` is deliberately unbound - a script
      should not be able to kill the device outright.

- [x] `tactility.wifi` - is_available/radio_state/station_state/ip_address/ssid/rssi/
      scan/is_scanning/scan_results. **12 tests.**

      States are strings (`"connected"`, `"turning_on"`) rather than numbers, so script
      comparisons read properly. `scan()` starts the scan and returns immediately - it
      takes seconds, and blocking would stall whatever task the script runs on. Connecting
      is unbound for now: it needs a password, so exposing it means deciding how a script
      obtains credentials and whether it may join arbitrary networks unprompted.

Both auto-find the first active device of their type rather than making scripts do the
find/release dance for a battery percentage. The reference is taken and released inside
each call, so nothing is pinned in between.

**Remaining, deferred until after the LVGL work** - see the note below:

- [ ] `tactility.gpio` - needs `GpioDescriptor` and driver-API plumbing, so more work than
      the others; a good second test of whether the binding ergonomics hold up
- [ ] `tactility.audio`, `tactility.display`, `tactility.bluetooth`, `tactility.lora`,
      `tactility.i2c`, `tactility.uart`, ... - the long tail toward ELF parity

### Order: demo-useful bindings first, full parity later

Full kernel parity is a long grind, and none of it is visible until something can draw.
The useful ordering is: bind what a demo app would actually show (done - log, device, fs,
settings, time, power, wifi), then move to checkpoints 6 and 7 so there is a real Lua app
to look at, and come back to finish parity afterwards with something concrete to test
against.

**Watch for:** anything blocking. A `lua_State` is single-threaded, and a blocking call
from the LVGL task will freeze the UI. Decide per binding whether it blocks, returns a
status, or takes a callback - and write it down in the header.

**Done when:** each has tests in the style of `BindingsTest.cpp`, including the failure
paths, and passes under ASan.

---

## Checkpoint 6 - LVGL bindings (`luavgl-module`) `[x]`

The big one, and where the project becomes visibly real.

Separate module from `lua-module` so an LVGL bump touches one place and `lua-module` stays
useful headless.

[luavgl](https://github.com/XuNeo/luavgl) (MIT) is the reference. It builds against **LVGL
9.1** (its `deps/lvgl` submodule on `master`), so the gap to this project's 9.4.0 is small.

*Earlier notes here claimed 8.x. That was wrong* - inferred from `lv_disp_t` in `disp.c`
and `LV_VERSION_CHECK(8, 3, 0)` guards, both of which are backward-compat leftovers rather
than the target version. The mistake overstated the porting cost; it did not change the
decision to write fresh, which rests on the lifetime model rather than the API era.

Worth taking from luavgl rather than reinventing:

- **rotable** - read-only tables in flash for constants. On ESP32 this is the difference
  between constants costing nothing and costing tens of KB of heap.
- **the declarative `set{}` dispatcher** - `obj:set{ w = 100, align = ... }` instead of one
  binding per setter. This is why luavgl is ~20 files rather than ~200.
- **userdata/`lv_obj_t` lifetime coupling via `LV_EVENT_DELETE`** - the genuinely hard part.
  LVGL deletes children with their parent; Lua's GC has no idea. Get this wrong and you get
  use-after-free that looks random. The `DeviceHandle` pattern in `device.cpp` is the same
  shape and a reasonable model.

Worth rewriting rather than porting: the display/input layer (renamed in 9.x), styles, and
font/asset loading - that should go through `lvgl-module` and the kernel filesystem rather
than luavgl's own `fs.c`.

Suggested slice order:

- [x] 6a - `lvgl.Object`, `Label`, `Button`, the `set{}` dispatcher, lifetime coupling.
      **28 tests, clean under ASan.** Written fresh against LVGL 9.4 rather than ported;
      luavgl was read for its design, not its code.

      What was taken from luavgl: a registry keyed by `lv_obj_t*` so one widget always
      yields one handle (`a == b` works), an `LV_EVENT_DELETE` callback that nulls the
      handle when LVGL frees the widget, and the declarative `set{}` dispatcher over a
      sorted name table instead of one binding per setter.

      What was changed:

      - **The registry holds handles weakly.** luavgl pins every handle for the life of
        the state; here a handle the script has dropped can be collected while the widget
        lives on.
      - **No `__gc` deleting widgets.** LVGL owns lifetime, so collecting a handle must
        not take the widget with it. A test asserts a widget survives its handle being
        collected.
      - **`luavgl_bindings_close()`.** The bug luavgl does not address: widgets outlive
        the runtime that made them (an app closes its script while its UI is on screen),
        and every wrapped widget holds a delete callback pointing at that `lua_State`.
        Tearing the tree down afterwards then calls into freed memory - found by ASan on
        the first run. Callers must detach before `lua_runtime_free()`.
      - **Unknown properties raise.** Silently ignoring `colour = ...` would leave a
        script looking correct while doing nothing.
      - **`set{ text = ... }` checks the widget class.** LVGL's setters do not, so writing
        text through a plain object would corrupt memory rather than fail.

      Also learned: **LVGL computes no geometry until a layout pass runs**, so reading a
      size straight after setting one gives 0. `obj:update_layout()` is bound for scripts
      that need the numbers before the next frame.

- [x] 6a-toolbar - `lvgl.toolbar.*` wrapping `lvgl-module`'s real toolbar rather than a
      Lua lookalike, so Lua apps get the same widget, styling and working close button as
      C++ apps. **47 tests, clean under ASan.**

      Getting the close button to work took five attempts on hardware and every one of
      them taught something worth keeping:

      1. **`set_nav_action` renders nothing if given an arbitrary string.** The icon is an
         LVGL symbol (`LV_SYMBOL_CLOSE`), not an image path. `lvgl.SYMBOL.*` is bound so
         scripts have real ones.
      2. **The toolbar centres itself in its parent.** A full-height sibling created after
         it simply covers it; the screen has to be a flex column with the toolbar first.
      3. **Callbacks live on internal children.** `lvgl_toolbar_set_nav_action()` installs
         on the toolbar's close button and returns nothing, so keying or detaching against
         the toolbar object silently misses. Both now search the subtree.
      4. **The deadlock.** `GuiService::hideApp()` takes the LVGL lock, and
         `lvgl_port_task` holds that lock across the whole of `lv_timer_handler()` -
         *including* `lv_async_call`. So a widget callback that stops its own app blocks
         the loader for 5 s ("Timed out waiting for hideApp() to complete") and the app is
         then torn down while Lua is still running in it. **Anything that closes an app
         must run off the LVGL task entirely** - the demo hands it to a short-lived thread.
      5. **`set_nav_action` adds rather than replaces.** A fresh toolbar already carries
         the default close handler, so setting one made both fire: the app closed, then
         the script's handler closed whatever was underneath. The binding now removes the
         existing handler first.

      Also: `pad_all` does not close the gap between flex children - `pad_row`/`pad_column`
      are bound for that, and without them a Lua screen looks visibly looser than a C++ one.

- [x] 6b - events and timers. **74 tests, clean under ASan.**

      `obj:on(lvgl.EVENT.CLICKED, fn)` / `obj:off(code)`, and
      `lvgl.Timer{ period, callback, repeat_count, paused }` with pause/resume/set_period/
      ready/delete.

      Handlers are queued with `lv_async_call` rather than run inside LVGL's dispatch, so a
      handler may delete its own widget - doing that mid-dispatch would corrupt the event
      loop. That deferral opens two gaps, both closed: the widget can be deleted before the
      dispatch runs (checked with `lv_obj_is_valid`), and the runtime can be freed
      (pending entries are cancelled by `luavgl_detach_all`).

      Unlike the toolbar's one-shot nav action, these fire repeatedly - a button is pressed
      many times over an app's life - so a handler stays registered until removed or its
      widget dies.

      **Timers are the sharpest lifetime hazard in the binding.** A timer has no widget to
      anchor it, so nothing else would notice its runtime had gone; it would simply keep
      firing into freed memory. Every timer a runtime creates is tracked and deleted by
      `luavgl_detach_all()`, so a script that forgets to stop one cannot leak it.

      Also found: **LVGL's `auto_delete` defaults to true**, so it frees an exhausted timer
      itself and leaves the Lua handle dangling - a crash on the first `repeat_count` test.
      Auto-delete is now off and the binding retires finite timers itself. LVGL has no
      repeat-count getter, so the count is tracked in the handle.
- [x] 6c - the core widget set. **98 tests, clean under ASan.**

      `Slider`, `Switch`, `Checkbox`, `Bar`, `Dropdown`, `Roller`, `TextArea`, `Line`,
      `Image`, `Spinner`, plus the properties they need (`value`, `min`, `max`, `options`,
      `checked`, `placeholder`, `one_line`, `password_mode`, `src`).

      Readers are methods (`value()`, `text()`, `is_checked()`) rather than a `get{}`
      mirror of `set{}` - that keeps the property table write-only, which is what makes a
      single flat table workable rather than needing luavgl's per-widget tables. Each
      raises on a widget with no such notion instead of returning something plausible.

      `options` takes a Lua array and builds LVGL's newline-separated string itself, and
      selections are 1-based in Lua against LVGL's 0-based, converted at the boundary.

      **Widget availability is per-target.** `CONFIG_LV_USE_SPINNER` is off in the ESP32
      sdkconfig, so `lv_spinner_create()` compiles on the simulator and fails on device -
      caught only because both targets get built. `lvgl.Spinner` uses `lvgl-module`'s
      portable one instead. Worth checking sdkconfig before binding any further widget.
- [x] 6d - styles and fonts. **115 tests, clean under ASan.**

      `lvgl.Style{...}` with `add_style`/`remove_style` on every widget, a selector
      argument (`lvgl.PART.KNOB | lvgl.STATE.PRESSED`) reaching parts and states the flat
      property table cannot, and `text_font` on both widgets and styles.

      Fonts are **named** (`'small'`/`'default'`/`'large'`/`'icon'`), resolved through
      `lvgl-module`'s density-aware `lvgl_get_text_font()`. Exposing raw `lv_font_t*` would
      tie a script to one display size.

      **The lifetime problem, and why it needed more than pinning.** `lv_obj_add_style()`
      stores the `lv_style_t*` - it does not copy - and unlike a widget handle there is no
      LVGL event to catch a stale one. Styles are pinned in the registry so collecting the
      Lua handle does not free them, but that alone is not enough: `lua_close()` frees the
      userdata regardless, so `luavgl_detach_all()` must strip every style off every live
      widget *before* the state closes. Verified by commenting the removal out and
      confirming ASan reports the use-after-free inside `lv_obj_update_layout` - the test
      catches the real bug, not just its own setup.

      Removal is per-style-pointer rather than clearing the widget with
      `LV_STYLE_PROP_ANY`: a widget may also carry styles a C++ app applied, and those are
      not the runtime's to strip.

      Note for future work: adding a source file does **not** make `idf.py build` pick it
      up - the CMake glob is cached, so an `idf.py reconfigure` is needed first. Cost a
      confusing round of undefined-reference errors.

**Biggest risk in the whole plan:** this is an ongoing maintenance burden tracking upstream
LVGL, on a project supporting 40+ devices. 6a is the checkpoint where it is still cheap to
walk away.

**Done when:** a `.lua` file draws a label and a working button on the simulator and on
hardware, and closing it leaks nothing. **6a is done on hardware** - `Apps/LuaSdkTest`
builds its whole UI from Lua, including the toolbar, and closes cleanly.

### Open: launch count stuck at 2

`tactility.settings` reads and writes correctly the first time but the counter does not
advance past 2 on device, so the write is not reaching storage after the first run. The
simulator tests pass, so it is device-specific - most likely the atomic
write-to-temp-then-`rename()` in `settings.cpp`, since FAT may reject a rename onto an
existing file. Worth checking `store()`'s return value on device before assuming.

---

## Checkpoint 7 - the Lua app runtime `[x]`

What makes a `.lua` file an app rather than a script.

- [x] Add an optional `app.runtime` key to the manifest (`elf` default, `lua`). Additive,
      so `AppManifestParsingV2` handles it - no V3 needed.

      Read directly from the map rather than through `getValueFromManifest()`, which logs
      an error for a missing key - correct for required keys, but every ELF manifest would
      trip it. An unrecognised value **fails** rather than falling back to ELF: a typo
      would otherwise start a Lua app as an ELF one and report a missing binary, a long way
      from the cause.
- [x] `createLuaApp(manifest)` alongside `createElfApp`, branching in `AppInstance.h`.

      Deliberately **outside** the `#ifdef ESP_PLATFORM` that guards `createElfApp`: a Lua
      app needs an interpreter, not a loader, so it runs on the simulator too.
- [x] Map the app lifecycle onto Lua callbacks. The script defines optional globals
      `on_create` / `on_show` / `on_hide` / `on_destroy`.

      Needed two new calls in `lua-module`: `lua_runtime_has_function()` and
      `lua_runtime_call()`. Globals rather than a returned table, because
      `lua_runtime_eval_file()` already runs the chunk for its side effects and discards
      its result - capturing a table would have changed that API.
- [x] Pass the app's install directory as `...`. `lua_runtime_eval_file()` already took an
      `arg` vararg, so this was free.
- [x] Teardown: close the runtime on destroy, LVGL objects and timers first.

      **The ordering is the whole story.** Widgets and timers routinely outlive the app -
      the loader tears the screen down *after* `onDestroy` - and each carries a callback
      into the `lua_State`. `luavgl_bindings_close()` must therefore run before
      `lua_runtime_free()`.

      `onDestroy` also has to take the LVGL lock itself. `onShow`/`onHide` are called by
      GuiService with it already held (`GuiService.cpp:356`), but `onCreate`/`onDestroy`
      run on the loader thread with no lock (`Loader.cpp:248,262`). Getting this backwards
      is exactly the deadlock class from checkpoint 6.
- [x] **Verified on hardware** (m5stack-tab5): `LuaHello.app` installs, launches, closes
      and relaunches cleanly, with working timers and touch events. The teardown ordering
      holds across repeated launch/exit cycles.

**Depends on:** checkpoint 6 (no UI, no app) and checkpoint 4 (this is what makes the flash
cost real).

**Done when:** a `.lua` app installed under `/data/app/<id>/` appears in the launcher, runs,
and exits cleanly, with no leak across repeated launches.

### How a Lua app gets packaged

An `.app` file is a USTAR tar built by `tactility.py`'s `package_all()`, containing:

```
manifest.properties
elf/<platform>.elf     # one per target platform
assets/                # optional, copied verbatim
```

A Lua app fits this with almost no change - the natural shape is:

```
manifest.properties    # app.runtime=lua
lua/main.lua           # entry point
assets/                # unchanged
```

Notable consequences, all in the good direction:

- **No `elf/` directory, so no per-platform builds.** One `.app` runs on every device
  whose firmware has `lua-module`. `target.platforms` stops meaning "we compiled for
  these" and starts meaning "we are known to work on these".
- **No ESP-IDF needed to build one.** Packaging becomes "tar up three things", which
  means the barrier to writing an app drops to owning a text editor. That is most of the
  point of this whole exercise.
- `install`/`run` over HTTP, the CDN, and the on-device unpacking should all be unchanged,
  since they only care about the tar.

Work needed in `tactility.py`, roughly in order:

- [x] Detect a Lua app (`app.runtime=lua` in the manifest) - `is_lua_app()`
- [x] Skip `build_all()` entirely for those; `package_intermediate_lua()` copies `lua/`
      instead of `elf/`.

      Also skips the `tool.json` refresh for a plain `build`, which was a network call on
      the way to packaging. Requiring the network to tar up a script would have undercut
      the point. `install`/`run` still refresh it, since they do talk to the CDN.

      `package_name()` needed a Lua branch too - it derived the filename from the ELF,
      which no longer exists, so it uses the app's folder name (`LuaHello.app`), matching
      what the ELF basename amounts to for a normal app.

      `install_action()` also needed one: it checked for a per-platform ELF before
      uploading and would have refused to install a Lua app that packaged fine.
- [x] Syntax-check scripts at package time, via `luac -p`.

      **Optional by design**: when no `luac` is on PATH it warns and continues. Requiring
      one would put a toolchain back in front of writing an app, which is most of the point
      of Lua support. Verified it both accepts valid scripts and fails broken ones with the
      compiler's message.
- [x] Decided about bytecode: **ship source**. Bytecode is not portable between simulator
      and ESP32 (different number sizes - see checkpoint 1) nor guaranteed across Lua
      versions. Revisit only if parse time actually hurts.

**Open question worth settling early:** a hybrid app - `elf/` *and* `lua/` in one package,
native code plus scripts. meshpunk does this. It is not obviously needed, and supporting it
means the runtime question stops being a simple either/or. Default to not supporting it
until something concrete wants it.

---

## Checkpoint 8 - developer experience `[ ]`

- [ ] Document the Lua API - generated from the binding tables if practical, so it cannot
      drift from the code
- [ ] An example app in `TactilityApps`, minimal and commented
- [ ] `LuaPlay` was an earlier run at this idea, done entirely inside one app: it carries
      its own Lua 5.3 (818 KB of sources) and its own `lua_lvgl.c`. Once checkpoint 6
      lands it can drop both and use the firmware's interpreter, which is the concrete
      payoff of the symbols export and a good real-world test. Worth reading first for
      what went wrong the first time - particularly how it handled LVGL object lifetimes.
- [ ] Decide on error surfacing: a script error currently goes to the log. It should
      probably reach the user, the way `ElfApp` shows an alert dialog.
- [ ] **Catch unexported symbols at package time.** An app compiles against SDK headers
      that declare much more than the module symbol tables export, so calling an
      unexported function only fails when the device tries to load the app -
      `Can't find common <name>`. `tactility.py` could read the ELF's undefined symbols
      (`nm -u`) and check them against the tables shipped in the SDK, failing the build
      with the offending names instead. That needs the tables in a machine-readable form
      beside the archives; generating a plain list per module during `release-sdk.py`
      would be enough. Turns a confusing on-device failure into a build error naming the
      function.

---

## Checkpoint 9 - distribution `[ ]`

Only worth doing if Lua apps get real use.

- [ ] Decide whether Lua apps install through the existing app mechanism unchanged (they
      should - `Location::external` and the manifest already cover it)
- [ ] Consider an on-device app store. meshpunk's `catalog.toml` + `min_fw` gating is a
      good model. Trust is PR review, no signing - same as their model and as ELF apps today.

---

## Unrelated things noticed along the way

Not part of this work, but found while doing it and worth not losing.

### esp_lvgl_port is nearly, but not quite, dead weight

`lvgl-module` has taken over displays and input devices, so most of
`espressif/esp_lvgl_port` (989 KB managed component) is unused. Four calls remain, all in
`Modules/lvgl-module/source/arch/lvgl_esp32.c`:

| Call | What it provides |
| --- | --- |
| `lvgl_port_init` / `lvgl_port_deinit` | the LVGL task and tick timer |
| `lvgl_port_lock` / `lvgl_port_unlock` | the recursive mutex behind `lvgl_lock()` |

Replacing them is roughly 80 lines in `lvgl-module`: a task calling `lv_timer_handler()` on
a period, a recursive mutex, and `lv_tick_set_cb`. That would drop the dependency entirely
and put the LVGL task under the same control as everything else the module already owns.

Worth doing, but it is firmware surgery on every device's display path - do it on its own,
with hardware to hand, not folded into unrelated work.

### The simulator renders a black window

Confirmed **not** caused by anything here: the same failure appears in current GitHub
Actions builds of the simulator. It launches, the window opens, contents stay black.

Consequence for this project: the simulator is currently only useful for *tests*, not for
looking at UI. That matters most at checkpoint 6, where the whole point is seeing widgets -
plan on testing LVGL bindings on hardware, and treat any simulator rendering as a bonus.

Worth a separate bug report upstream with the Actions build as evidence, since it is
reproducible outside this tree.

---

## Things deliberately not being done

- **No Lua sandbox.** `luaL_openlibs` opens everything. ELF apps already run unsandboxed,
  so a Lua-only sandbox would not match the platform's trust model. The real limits are
  practical: no `os.execute`/`io.popen` (no process model), no `package.loadlib` (no `dlopen`).
- **No hybrid Lua/ELF apps** in the meshpunk style. Two parallel runtimes, each whole.
  Revisit only if something concrete needs it.
- **Not using `espressif/lua`.** It is ESP-IDF only; vendoring gives the simulator the same
  interpreter from the same source with no version skew.

---

## Quick reference

```bash
# After adding a source file, ESP-IDF needs a reconfigure - the CMake glob is cached,
# and the symptom is an "undefined reference" to something that plainly exists:
#   idf.py reconfigure && idf.py build

# Simulator (WSL or Linux - not native Windows)
cmake -B buildsim -G Ninja && ninja -C buildsim build-tests
cd buildsim && ctest --test-dir Tests

# Just the Lua tests
ninja -C buildsim LuaModuleTests && ./buildsim/Tests/lua-module/LuaModuleTests

# Under AddressSanitizer - worth it for anything touching handles or the GC
cmake -B asanbuild -G Ninja -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="-fsanitize=address -g" \
  -DCMAKE_C_FLAGS="-fsanitize=address -g" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address"

# ESP32 (Windows: source the IDF profile first)
idf.py build
python Buildscripts/release-sdk-current.py   # SDK, into release/
```

Regenerating `symbols.c` after a Lua update:

```bash
ninja -C buildsim lua                              # any build producing liblua.a will do
python Modules/lua-module/generate-symbols.py
```

Rewrites the file in place - nothing is added by hand. It takes the intersection of what
the public headers declare and what the archive defines, and only rewrites the block
between its `GENERATED BEGIN`/`END` markers, so `lua-module`'s own curated API above them
survives.

The intersection matters in both directions: declared-but-undefined names are macros
(`lua_pcall`, `lua_call`, `lua_tostring`, `luaL_loadbuffer`, and since 5.5 `luaL_openlibs`)
which resolve at compile time inside the app and have no symbol to export, while
defined-but-undeclared names are interpreter internals no app should reach.

Two guards, both verified by deliberately breaking them: the script refuses to write if it
finds fewer than 150 symbols (a stale or partial archive) or if the markers are missing,
leaving the file untouched in either case. `ModuleTest.cpp` then fails if the committed
table drops below 150 entries.

Re-running it on an unchanged tree is a no-op, so it is safe to run any time.
