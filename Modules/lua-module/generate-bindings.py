#!/usr/bin/env python3
"""
Generates Lua wrappers from typed symbol signatures.

Demonstrates the proposal in GENERATION.md: annotate a kernel function with its signature
once, and the wrapper that exposes it to Lua is derived rather than written.

    # writes source/bindings/gpio_generated.cpp
    python Modules/lua-module/generate-bindings.py bindings/gpio.signatures

    # explicit destination
    python Modules/lua-module/generate-bindings.py bindings/gpio.signatures /tmp/out.cpp

    # stdout, for a quick look
    python Modules/lua-module/generate-bindings.py bindings/gpio.signatures -

The input is a small text file rather than a parse of ModuleSymbol tables, so the idea can
be judged without first changing a kernel header. The signature strings are exactly what
DEFINE_MODULE_SYMBOL_TYPED() would carry.

Signature grammar:

    <return><name>(<param>,...)      e.g. e(pGpioDescriptor,*b)

    v                void
    b                bool
    i8 i16 i32 i64   signed integers
    u8 u16 u32 u64   unsigned integers
    f                float/double
    s                const char*
    e                error_t          (becomes the nil,message convention)
    p<Name>          struct Name*     (userdata with a metatable)
    *<code>          out-parameter    (becomes an extra return value)
"""

import os
import re
import sys

# code -> (C type, how to read it from Lua, how to push it to Lua)
SCALARS = {
    "b":   ("bool",        "lua_toboolean(state, {i}) != 0",                        "lua_pushboolean(state, {v})"),
    "i8":  ("int8_t",      "(int8_t)luaL_checkinteger(state, {i})",                 "lua_pushinteger(state, {v})"),
    "i16": ("int16_t",     "(int16_t)luaL_checkinteger(state, {i})",                "lua_pushinteger(state, {v})"),
    "i32": ("int32_t",     "(int32_t)luaL_checkinteger(state, {i})",                "lua_pushinteger(state, {v})"),
    "i64": ("int64_t",     "(int64_t)luaL_checkinteger(state, {i})",                "lua_pushinteger(state, {v})"),
    "u8":  ("uint8_t",     "(uint8_t)luaL_checkinteger(state, {i})",                "lua_pushinteger(state, {v})"),
    "u16": ("uint16_t",    "(uint16_t)luaL_checkinteger(state, {i})",               "lua_pushinteger(state, {v})"),
    "u32": ("uint32_t",    "(uint32_t)luaL_checkinteger(state, {i})",               "lua_pushinteger(state, {v})"),
    "u64": ("uint64_t",    "(uint64_t)luaL_checkinteger(state, {i})",               "lua_pushinteger(state, {v})"),
    "f":   ("double",      "(double)luaL_checknumber(state, {i})",                  "lua_pushnumber(state, {v})"),
    "s":   ("const char*", "luaL_checkstring(state, {i})",                          "lua_pushstring(state, {v})"),
}


class UnsupportedSignature(Exception):
    """Raised for a signature the generator deliberately refuses to handle."""


def camel_to_snake(name):
    """GpioDescriptor -> gpio_descriptor, matching the kernel's C naming."""
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


class Param:
    def __init__(self, code):
        self.is_out = code.startswith("*")
        body = code[1:] if self.is_out else code

        if body.startswith("p"):
            self.kind = "pointer"
            self.type_name = body[1:]
            if not self.type_name:
                raise UnsupportedSignature("pointer parameter has no type name")
            self.c_type = f"struct {self.type_name}*"
        elif body in SCALARS:
            self.kind = "scalar"
            self.code = body
            self.c_type = SCALARS[body][0]
        else:
            raise UnsupportedSignature(f"unknown parameter type '{code}'")

        if self.is_out and self.kind == "pointer":
            raise UnsupportedSignature("out-parameters returning handles need a lifetime "
                                       "policy, so they are written by hand")


def parse_signature(signature):
    match = re.fullmatch(r"([a-z]\w*)\(([^)]*)\)", signature.strip())
    if not match:
        raise UnsupportedSignature(f"malformed signature '{signature}'")

    return_code, param_text = match.group(1), match.group(2).strip()
    if return_code not in ("v", "e", "b") and return_code not in SCALARS:
        raise UnsupportedSignature(f"unsupported return type '{return_code}'")

    params = [Param(p.strip()) for p in param_text.split(",") if p.strip()]
    return return_code, params


def generate(name, signature):
    """Returns the C source for one wrapper."""
    return_code, params = parse_signature(signature)

    lines = [f"static int lua_{name}(lua_State* state) {{"]
    arguments = []
    lua_index = 1
    outputs = []

    for index, param in enumerate(params):
        variable = f"argument{index}"
        if param.is_out:
            c_type = param.c_type
            lines.append(f"    {c_type} {variable} = ({c_type})0;")
            arguments.append(f"&{variable}")
            outputs.append((param, variable))
        elif param.kind == "pointer":
            # Deliberately a call into hand-written code rather than a cast. A handle is not
            # a bare pointer: DeviceHandle carries a `released` flag so an explicit release()
            # followed by collection does not double-free, and the accessor is what raises
            # when a script uses a handle it already gave up. No signature can express that,
            # so generation stops at the boundary and calls the code that owns the policy.
            accessor = f"lua_bindings_check_{camel_to_snake(param.type_name)}"
            lines.append(f"    auto* {variable} = {accessor}(state, {lua_index});")
            arguments.append(variable)
            lua_index += 1
        else:
            reader = SCALARS[param.code][1].format(i=lua_index)
            lines.append(f"    const {param.c_type} {variable} = {reader};")
            arguments.append(variable)
            lua_index += 1

    call = f"{name}({', '.join(arguments)})"
    lines.append("")

    if return_code == "e":
        # error_t maps onto Lua's "nil plus a reason" convention, which is what the
        # hand-written bindings already do - so generated ones need no special case.
        lines.append(f"    const error_t error = {call};")
        lines.append("    if (error != ERROR_NONE) {")
        lines.append("        lua_pushnil(state);")
        lines.append("        lua_pushstring(state, error_to_string(error));")
        lines.append("        return 2;")
        lines.append("    }")
        lines.append("")

        if not outputs:
            # Nothing to report but success. `true` keeps `assert(f())` working.
            lines.append("    lua_pushboolean(state, 1);")
            lines.append("    return 1;")
        else:
            for param, variable in outputs:
                lines.append("    " + SCALARS[param.code][2].format(v=variable) + ";")
            lines.append(f"    return {len(outputs)};")
    elif return_code == "v":
        lines.append(f"    {call};")
        lines.append("    return 0;")
    else:
        c_type = SCALARS[return_code][0] if return_code in SCALARS else "bool"
        pusher = SCALARS[return_code][2] if return_code in SCALARS else "lua_pushboolean(state, {v})"
        lines.append(f"    const {c_type} result = {call};")
        lines.append("    " + pusher.format(v="result") + ";")
        lines.append("    return 1;")

    lines.append("}")
    return "\n".join(lines)


def lua_name(c_name, strip_prefix):
    return c_name[len(strip_prefix):] if strip_prefix and c_name.startswith(strip_prefix) else c_name


def default_output_path(signature_path):
    """bindings/gpio.signatures -> source/bindings/gpio_generated.cpp"""
    module_root = os.path.dirname(os.path.dirname(os.path.abspath(signature_path)))
    stem = os.path.splitext(os.path.basename(signature_path))[0]
    return os.path.join(module_root, "source", "bindings", f"{stem}_generated.cpp")


def main():
    if len(sys.argv) < 2:
        print(__doc__.strip())
        return 2

    path = sys.argv[1]

    # Written to a file by default, next to the hand-written bindings. `-` writes to stdout
    # instead, which is what the compile check and a quick eyeball both want.
    if len(sys.argv) >= 3:
        output_path = sys.argv[2]
    else:
        output_path = default_output_path(path)

    with open(path, "r", encoding="utf-8") as handle:
        raw = handle.read()

    table_name = "gpio"
    strip_prefix = ""
    includes = []
    entries = []
    skipped = []

    for line in raw.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("@table "):
            table_name = line[len("@table "):].strip()
            continue
        if line.startswith("@prefix "):
            strip_prefix = line[len("@prefix "):].strip()
            continue
        if line.startswith("@include "):
            includes.append(line[len("@include "):].strip())
            continue

        name, _, signature = line.partition(" ")
        entries.append((name.strip(), signature.strip()))

    wrappers = []
    registrations = []
    for name, signature in entries:
        try:
            wrappers.append(generate(name, signature))
            registrations.append((lua_name(name, strip_prefix), f"lua_{name}"))
        except UnsupportedSignature as error:
            skipped.append((name, str(error)))

    out = []
    out.append("// GENERATED by Modules/lua-module/generate-bindings.py - do not edit.")
    out.append(f"// Source: {os.path.basename(path)}")
    out.append("")
    out.append('#include "bindings_private.h"')
    out.append("")
    for include in includes:
        out.append(f"#include <{include}>")
    out.append("")
    out.append('extern "C" {')
    out.append("#include <lauxlib.h>")
    out.append("}")
    out.append("")
    out.append("namespace {")
    out.append("")
    out.append("\n\n".join(wrappers))
    out.append("")
    out.append("const luaL_Reg functions[] = {")
    for lua_fn, c_fn in registrations:
        out.append(f'    {{ "{lua_fn}", {c_fn} }},')
    out.append("    { nullptr, nullptr }")
    out.append("};")
    out.append("")
    out.append("} // namespace")
    out.append("")
    out.append(f"void lua_bindings_open_{table_name}(lua_State* state) {{")
    out.append("    luaL_newlib(state, functions);")
    out.append(f'    lua_setfield(state, -2, "{table_name}");')
    out.append("}")
    out.append("")

    content = "\n".join(out)

    if output_path == "-":
        print(content)
    else:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        # newline="\n" so the file is identical on Windows and Linux - otherwise a
        # regeneration on the other platform shows up as a whole-file diff.
        with open(output_path, "w", encoding="utf-8", newline="\n") as handle:
            handle.write(content + "\n")
        # stderr, so `... gpio.signatures -` and a redirect of this branch both produce
        # only C++ on stdout
        print(f"Wrote {os.path.relpath(output_path)}", file=sys.stderr)

    if skipped:
        print(f"{len(skipped)} not generated, written by hand instead:", file=sys.stderr)
        for name, reason in skipped:
            print(f"  {name}: {reason}", file=sys.stderr)

    print(f"Generated {len(registrations)} of {len(entries)} functions", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
