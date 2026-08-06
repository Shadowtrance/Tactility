#!/usr/bin/env python3
"""
Check which symbols an ELF app needs that the firmware does not export.

Apps are relocated at load time against the firmware's symbol tables. A symbol the app references
but the firmware does not export fails at *load* time with a bare

    E (9713) ELF: Can't find common xPortEnterCriticalTimeout
    E (9716) ElfApp: Application failed to load: missing symbol

which names one symbol per attempt, so finding them all by flashing is a slow loop. This reads the
app's ELF directly and reports every missing symbol at once.

Usage:
    python Buildscripts/check-app-symbols.py <path/to/App.app.elf> [--sdk <TactilityWork path>]
    python Buildscripts/check-app-symbols.py --list-exports

Exit status is 1 when symbols are missing, so this can gate a build.
"""

import argparse
import re
import struct
import sys
from pathlib import Path

# Where the firmware declares what apps may link against. Both mechanisms are consulted at load
# time (see tt_symbol_resolver in TactilityC/Source/tt_init.cpp, which falls through to
# module_resolve_symbol_global), so a symbol in either table resolves.
EXPORT_SOURCES = [
    ("TactilityKernel/source/symbols.c", "DEFINE_MODULE_SYMBOL"),
    ("Modules/lvgl-module/source/symbols.c", "DEFINE_MODULE_SYMBOL"),
    ("Modules/lua-module/source/symbols.c", "DEFINE_MODULE_SYMBOL"),
    ("TactilityC/Source/tt_init.cpp", "ESP_ELFSYM_EXPORT"),
    ("TactilityC/Source/symbols/*.cpp", "ESP_ELFSYM_EXPORT"),
    ("Drivers/*/source/symbols.c", "DEFINE_MODULE_SYMBOL"),
]


def find_firmware_root(explicit):
    """Locates the TactilityWork checkout holding the export tables."""
    if explicit:
        return Path(explicit)

    # Default to the sibling checkout next to this apps repo, then to this script's own tree.
    here = Path(__file__).resolve()
    candidates = [
        here.parent.parent,
        here.parent.parent.parent / "TactilityWork",
    ]
    for candidate in candidates:
        if (candidate / "TactilityKernel/source/symbols.c").is_file():
            return candidate
    raise SystemExit(
        "Could not locate TactilityWork. Pass --sdk <path> explicitly."
    )


def collect_exports(root):
    """Returns the set of symbol names the firmware exports to apps."""
    exports = set()
    for pattern, macro in EXPORT_SOURCES:
        matcher = re.compile(re.escape(macro) + r"\(\s*([A-Za-z_][A-Za-z_0-9]*)\s*\)")
        if "*" in pattern:
            paths = sorted(root.glob(pattern))
        else:
            candidate = root / pattern
            paths = [candidate] if candidate.is_file() else []

        for path in paths:
            try:
                text = path.read_text(encoding="utf-8", errors="replace")
            except OSError:
                continue
            # Skip commented-out entries so a disabled export isn't reported as available.
            for line in text.splitlines():
                stripped = line.lstrip()
                if stripped.startswith("//") or stripped.startswith("*"):
                    continue
                exports.update(matcher.findall(line))
    return exports


def read_undefined_symbols(elf_path):
    """Returns the set of SHN_UNDEF symbol names in a 32-bit little-endian ELF."""
    data = elf_path.read_bytes()

    if data[:4] != b"\x7fELF":
        raise SystemExit(f"{elf_path} is not an ELF file")
    if data[4] != 1:
        raise SystemExit(f"{elf_path} is not 32-bit; this script handles ELF32 only")

    section_header_offset = struct.unpack_from("<I", data, 0x20)[0]
    section_header_size = struct.unpack_from("<H", data, 0x2E)[0]
    section_count = struct.unpack_from("<H", data, 0x30)[0]
    section_name_index = struct.unpack_from("<H", data, 0x32)[0]

    sections = []
    for index in range(section_count):
        offset = section_header_offset + index * section_header_size
        name, _type, _flags, _addr, off, size, link, _info, _align, _entsize = struct.unpack_from(
            "<10I", data, offset
        )
        sections.append({"name": name, "off": off, "size": size, "link": link})

    name_table_offset = sections[section_name_index]["off"]

    def section_name(offset):
        end = data.index(b"\0", name_table_offset + offset)
        return data[name_table_offset + offset : end].decode()

    for section in sections:
        section["label"] = section_name(section["name"])

    undefined = set()
    for section in sections:
        if section["label"] not in (".dynsym", ".symtab"):
            continue
        string_table = sections[section["link"]]
        entry_count = section["size"] // 16
        for index in range(entry_count):
            offset = section["off"] + index * 16
            name_offset, _value, _size, _info, _other, shndx = struct.unpack_from(
                "<IIIBBH", data, offset
            )
            # shndx == SHN_UNDEF means the symbol must come from the firmware at load time.
            if shndx != 0 or name_offset == 0:
                continue
            end = data.index(b"\0", string_table["off"] + name_offset)
            undefined.add(data[string_table["off"] + name_offset : end].decode())
    return undefined


def main():
    parser = argparse.ArgumentParser(
        description="Report symbols an ELF app needs that the firmware does not export."
    )
    parser.add_argument("elf", nargs="?", help="path to the built <App>.app.elf")
    parser.add_argument("--sdk", help="path to the TactilityWork checkout")
    parser.add_argument(
        "--list-exports",
        action="store_true",
        help="print every exported symbol and exit",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="also list the symbols that do resolve",
    )
    args = parser.parse_args()

    root = find_firmware_root(args.sdk)
    exports = collect_exports(root)

    if args.list_exports:
        for symbol in sorted(exports):
            print(symbol)
        return 0

    if not args.elf:
        parser.error("an ELF path is required unless --list-exports is given")

    elf_path = Path(args.elf)
    if not elf_path.is_file():
        raise SystemExit(f"No such file: {elf_path}")

    undefined = read_undefined_symbols(elf_path)
    missing = sorted(undefined - exports)
    resolved = sorted(undefined & exports)

    print(f"Firmware exports : {len(exports)}")
    print(f"App references   : {len(undefined)} undefined symbols")
    print(f"Resolved         : {len(resolved)}")
    print(f"Missing          : {len(missing)}")

    if args.verbose and resolved:
        print("\nResolved symbols:")
        for symbol in resolved:
            print(f"  {symbol}")

    if missing:
        print("\nMissing symbols (the app will fail to load):")
        for symbol in missing:
            print(f"  {symbol}")
        print(
            "\nExport these from the firmware, then rebuild it:\n"
            "  TactilityKernel/source/symbols.c      DEFINE_MODULE_SYMBOL(name)\n"
            "  TactilityC/Source/tt_init.cpp         ESP_ELFSYM_EXPORT(name)\n"
            "  TactilityC/Source/symbols/*.cpp       ESP_ELFSYM_EXPORT(name)   (grouped by area)\n"
            "\nNote that macros can hide the real name: portENTER_CRITICAL expands to\n"
            "xPortEnterCriticalTimeout/vPortExitCriticalMultiCore on multicore targets."
        )
        return 1

    print("\nAll symbols resolve.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
