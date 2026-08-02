#!/usr/bin/env python3
"""
Regenerates the Lua C API portion of source/symbols.c.

Run after updating Lua. Writes the file in place; no manual editing needed.

    python Modules/lua-module/generate-symbols.py

The exported set is the intersection of what the public headers *declare* and what the
built archive actually *defines*:

  - Declared-but-not-defined is normal and must be excluded. A large part of the Lua "API"
    is macros - lua_pcall, lua_call, lua_tostring, luaL_loadbuffer and, since 5.5,
    luaL_openlibs are all #defines over the functions below. They resolve inside the app at
    compile time and have no symbol to export.
  - Defined-but-not-declared is internal to the interpreter and none of an app's business.

lua-module's own API is listed separately in symbols.c and left alone by this script: it is
a curated set, not everything the headers happen to declare.

Requires a built liblua.a. Either target works - the exported names are the same - so an
existing simulator build is enough and there is no need to build for ESP32 first.
"""

import os
import re
import subprocess
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SYMBOLS_FILE = os.path.join(REPO_ROOT, "Modules", "lua-module", "source", "symbols.c")
INCLUDE_DIR = os.path.join(REPO_ROOT, "Libraries", "lua", "Include")
PUBLIC_HEADERS = ("lua.h", "lauxlib.h", "lualib.h")

# Everything between these markers is rewritten; everything outside is preserved.
BEGIN_MARKER = "    // GENERATED BEGIN - see generate-symbols.py"
END_MARKER = "    // GENERATED END"

# Where a built archive might be, newest first at call time
ARCHIVE_CANDIDATES = (
    os.path.join("build", "esp-idf", "lua", "liblua.a"),
    os.path.join("buildsim", "Libraries", "lua", "liblua.a"),
    os.path.join("buildasan", "Libraries", "lua", "liblua.a"),
)

GROUPS = (
    ("lua core", lambda name: name.startswith("lua_")),
    ("auxiliary library", lambda name: name.startswith("luaL_")),
    ("standard libraries", lambda name: name.startswith("luaopen_")),
)


def fail(message):
    print(f"error: {message}", file=sys.stderr)
    sys.exit(1)


def find_archive():
    found = [path for path in ARCHIVE_CANDIDATES if os.path.isfile(os.path.join(REPO_ROOT, path))]
    if not found:
        fail(
            "no liblua.a found - build first, e.g.\n"
            "  cmake -B buildsim -G Ninja && ninja -C buildsim lua"
        )
    # Newest wins, so a fresh build is used even when older ones linger
    found.sort(key=lambda p: os.path.getmtime(os.path.join(REPO_ROOT, p)), reverse=True)
    return os.path.join(REPO_ROOT, found[0])


def declared_names():
    """Every lua_/luaL_/luaopen_ identifier appearing in the public headers."""
    pattern = re.compile(r"\b(?:lua|luaL|luaopen)_[a-zA-Z0-9_]+")
    names = set()

    for header in PUBLIC_HEADERS:
        path = os.path.join(INCLUDE_DIR, header)
        if not os.path.isfile(path):
            fail(f"missing public header: {path}")
        with open(path, "r", encoding="utf-8", errors="replace") as handle:
            names.update(pattern.findall(handle.read()))

    return names


def defined_names(archive):
    """Names the archive defines as text symbols."""
    for tool in ("nm", "riscv32-esp-elf-nm", "xtensa-esp32-elf-nm"):
        try:
            result = subprocess.run(
                [tool, "--defined-only", archive],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True,
            )
        except (FileNotFoundError, subprocess.CalledProcessError):
            continue

        names = set()
        for line in result.stdout.decode(errors="replace").splitlines():
            parts = line.split()
            # "<address> T <name>"; T is an exported text symbol
            if len(parts) == 3 and parts[1] == "T":
                names.add(parts[2])
        return names

    fail("no usable nm found (tried nm, riscv32-esp-elf-nm, xtensa-esp32-elf-nm)")


def render(names):
    lines = [BEGIN_MARKER]

    remaining = set(names)
    for index, (title, belongs) in enumerate(GROUPS):
        group = sorted(name for name in remaining if belongs(name))
        remaining.difference_update(group)
        if not group:
            continue
        if index > 0:
            lines.append("")
        lines.append(f"    // {title}")
        lines.extend(f"    DEFINE_MODULE_SYMBOL({name})," for name in group)

    if remaining:
        lines.append("")
        lines.append("    // other")
        lines.extend(f"    DEFINE_MODULE_SYMBOL({name})," for name in sorted(remaining))

    lines.append(END_MARKER)
    return "\n".join(lines)


def main():
    archive = find_archive()
    exported = sorted(declared_names() & defined_names(archive))

    if len(exported) < 150:
        fail(
            f"only {len(exported)} symbols found, expected 150+.\n"
            f"The archive at {archive} is probably stale or built without the full library."
        )

    with open(SYMBOLS_FILE, "r", encoding="utf-8") as handle:
        content = handle.read()

    if BEGIN_MARKER not in content or END_MARKER not in content:
        fail(f"markers not found in {SYMBOLS_FILE} - it must contain the GENERATED block")

    before, _, rest = content.partition(BEGIN_MARKER)
    _, _, after = rest.partition(END_MARKER)

    with open(SYMBOLS_FILE, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(before + render(exported) + after)

    print(f"Wrote {len(exported)} Lua C API symbols to {os.path.relpath(SYMBOLS_FILE, REPO_ROOT)}")
    print(f"  archive: {os.path.relpath(archive, REPO_ROOT)}")
    print("  (lua-module's own API is listed above the generated block and left untouched)")


if __name__ == "__main__":
    main()
