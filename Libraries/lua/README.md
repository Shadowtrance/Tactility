# Lua

Vendored copy of [Lua](https://www.lua.org/) 5.5.0, licensed under the MIT license
(see `LICENSE-MIT.md`).

This builds for both ESP32 (as an ESP-IDF component) and the POSIX simulator, so that
`lua-module` has an identical interpreter on both targets.

## Layout

Upstream ships every source and header in a single `src/` directory. That is split here
to match the project convention:

- `Include/` — the public API: `lua.h`, `luaconf.h`, `lualib.h`, `lauxlib.h`, `lua.hpp`
- `Source/` — the implementation, plus the internal headers it includes privately

The standalone interpreter (`lua.c`) and bytecode compiler (`luac.c`) are omitted: both
define `main()` and neither is useful when embedding.

## Local changes

`Include/luaconf.h` is the only modified file. The changes are bracketed by
`TACTILITY BEGIN` / `TACTILITY END` comments so they are easy to find when updating:

- `LUA_USE_POSIX` (and `LUA_USE_DLOPEN`, `LUA_READLINELIB`) are undefined on ESP32, which
  drops `popen`, `system` and the other process-spawning parts of the standard library
  that have no meaning there.
- `LUA_32BITS` is defined on ESP32, making numbers 32-bit floats and integers. Xtensa and
  RISC-V ESP32 targets have no 64-bit FPU, so the default 64-bit doubles are slow and cost
  twice the memory per value. Note it is tested with `#if defined()`, so it must be left
  *undefined* elsewhere - defining it to 0 would enable it everywhere.

Espressif's own `espressif/lua` component makes the same `LUA_32BITS` choice, via header
injection rather than an edit. That approach is cleaner but ESP-only; this copy also has
to build for the POSIX simulator, so the bracketed edit covers both from one source.

## Updating

Download a new release from https://www.lua.org/ftp/, re-split it as described above, then
re-apply the `luaconf.h` changes and regenerate the symbol table:

```bash
ninja -C buildsim lua                              # or any build that produces liblua.a
python Modules/lua-module/generate-symbols.py
```

That rewrites `Modules/lua-module/source/symbols.c` in place. No entries are added by
hand: the script takes the intersection of what the public headers declare and what the
archive defines, and only touches the block between its `GENERATED BEGIN`/`END` markers -
`lua-module`'s own API sits above them and is curated, not generated.

Watch for API changes: 5.5 gave `lua_newstate` a third argument, retired
`lua_resetthread` and `lua_setcstacklimit`, and turned `luaL_openlibs` into a macro over
the new `luaL_openselectedlibs`. `Tests/lua-module` pins `LUA_VERSION_NUM`, so an
unintended version change fails there rather than silently shipping.

**Version changes are breaking for already-built apps.** The SDK exports these headers for
apps to compile against and resolves the matching symbols at load time, and Lua explicitly
does not promise binary compatibility between versions. Treat a Lua bump as an SDK version
bump.
