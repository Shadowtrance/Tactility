# Generating bindings from typed symbols

A response to the maintainability objection: 4,500 hand-written wrapper lines today, ~7,000
at parity, all of which have to move when the kernel API moves. This is a sketch of how the
mechanical part stops being hand-written.

## The idea

`ModuleSymbol` currently records a name and an address:

```c
struct ModuleSymbol { const char* name; const void* symbol; };
#define DEFINE_MODULE_SYMBOL(symbol) { #symbol, (void*)&symbol }
```

Nothing there says what the function *takes* or *returns*, so a wrapper cannot be derived
from it. Adding a signature is what makes generation possible:

```c
struct ModuleSymbol {
    const char* name;
    const void* symbol;
    /** Signature, or NULL for symbols that are not meant to be scripted. */
    const char* signature;
};

#define DEFINE_MODULE_SYMBOL(symbol) { #symbol, (void*)&symbol, NULL }
#define DEFINE_MODULE_SYMBOL_TYPED(symbol, sig) { #symbol, (void*)&symbol, sig }
```

The existing macro keeps working unchanged, so nothing has to be annotated at once and
nothing breaks if it never is.

## What a signature looks like

One string, borrowing dlang/JNI style: return type, then parameters.

```
e(pd,b)      error_t f(struct GpioDescriptor*, bool)
e(pd,*b)     error_t f(struct GpioDescriptor*, bool* out)
e(pd,*u32)   error_t f(struct GpioDescriptor*, uint32_t* out)
b(pd)        bool    f(struct GpioDescriptor*)
u32()        uint32_t f(void)
```

| Code | C type | Lua |
| --- | --- | --- |
| `v` | void | (nothing) |
| `b` | bool | boolean |
| `i32` `u32` `i64` `u64` | integers | integer |
| `f` | float/double | number |
| `s` | `const char*` | string |
| `e` | `error_t` | see below |
| `p<name>` | opaque pointer | userdata with a metatable |
| `*<code>` | out-parameter | extra return value |

Two conventions do most of the work:

- **`error_t` return becomes the Lua failure convention.** `ERROR_NONE` returns the
  out-parameters; anything else returns `nil, error_to_string(err)`. That is already what
  the hand-written bindings do, so generated ones match without a special case.
- **Out-parameters become return values**, in declaration order. `error_t f(d, bool* out)`
  becomes `local level, err = gpio.get_level(d)`.

## Worked example

`gpio_controller.h` is 16 functions of near-identical shape - one of the bindings still
unwritten, and a fair test:

```c
DEFINE_MODULE_SYMBOL_TYPED(gpio_descriptor_set_level,  "e(pGpioDescriptor,b)"),
DEFINE_MODULE_SYMBOL_TYPED(gpio_descriptor_get_level,  "e(pGpioDescriptor,*b)"),
DEFINE_MODULE_SYMBOL_TYPED(gpio_descriptor_set_flags,  "e(pGpioDescriptor,u32)"),
DEFINE_MODULE_SYMBOL_TYPED(gpio_descriptor_get_flags,  "e(pGpioDescriptor,*u32)"),
DEFINE_MODULE_SYMBOL_TYPED(gpio_descriptor_release,    "e(pGpioDescriptor)"),
```

generating, for `get_level`:

```c
static int lua_gpio_descriptor_get_level(lua_State* state) {
    auto* descriptor = (struct GpioDescriptor*)luavgl_check_userdata(state, 1, "GpioDescriptor");
    bool out_high = false;

    const error_t error = gpio_descriptor_get_level(descriptor, &out_high);
    if (error != ERROR_NONE) {
        lua_pushnil(state);
        lua_pushstring(state, error_to_string(error));
        return 2;
    }

    lua_pushboolean(state, out_high);
    return 1;
}
```

Roughly 15 lines of wrapper per function, replaced by one annotation on a line that already
exists. For gpio alone that is ~240 lines that never get written.

## What this does not solve

Being straight about the limits, because they are most of the remaining code:

- **Discovery and policy are not generatable.** `power.cpp` is 254 lines and barely any of
  it is marshalling - it finds a power supply that supports the property being asked about,
  because a board can register several `POWER_SUPPLY_TYPE` devices and they do not all
  answer the same questions. No signature expresses that.
- **Lifetime-bearing types still need hand-written code.** Anything returning a handle needs
  a metatable, a release path, and a decision about what happens when a script drops it.
- **Anything with a shape of its own.** `fs.list()` returns an array of tables;
  `time.date()` mirrors `os.date("*t")`. Those are API design, not translation.

Rough split of the current 4,500 lines: maybe half is mechanical enough to generate, and
the regular driver APIs (gpio, i2c, uart, pwm) are the most mechanical part - which is also
most of what is left to write.

The honest claim is not "generation removes the maintenance burden". It is that it removes
the part which is pure repetition, and turns an API change from "find and fix every wrapper"
into "the build fails where a signature no longer matches".

## Why put it in ModuleSymbol

The alternative is a separate manifest of signatures beside the tables. That drifts: nothing
forces the two to agree, and a stale entry produces a wrapper that compiles and then
misbehaves. Keeping the signature on the symbol means it lives next to the thing it
describes and moves with it.

It also serves more than Lua. The same table would let `tactility.py` check a side-loaded
app's undefined symbols at package time (checkpoint 8's `nm -u` idea), and would give any
future scripting layer the same starting point.

## Cost

- `ModuleSymbol` grows one pointer per entry: 4 bytes each. There are 1,026 symbols across
  every module today, so **4 KB** if all of them were annotated - but the field is `NULL`
  unless a module opts in, and the signature strings themselves only exist for annotated
  symbols. Annotating just the driver APIs worth scripting is a few hundred bytes.
- Existing `DEFINE_MODULE_SYMBOL` uses keep compiling with `signature = NULL`.
- The generator is a build-time Python script, in the same place as
  `generate-symbols.py`.

## Status

**Built and compiling.** `generate-bindings.py` turns
`bindings/gpio.signatures` into wrappers that compile against the real kernel headers:

```bash
python Modules/lua-module/generate-bindings.py Modules/lua-module/bindings/gpio.signatures
```

Measured on `gpio_controller.h`:

| | |
| --- | --- |
| Signature lines written by hand | **13** |
| C++ lines generated | **157** |
| Functions generated | **11 of 13** |
| Object code | 9,272 bytes, all 11 wrappers present |

The 2 it refuses are the right 2, and the generator says why rather than emitting something
that compiles and misbehaves:

```
gpio_descriptor_get_owner_type: unknown parameter type '*enum'
gpio_descriptor_add_callback:   unknown parameter type 'fn'
```

An enum out-parameter needs a name/value mapping so a script sees `"input"` rather than `2`;
a callback needs a registry, a hop off the ISR, and a lifetime policy. Both are real
decisions, so they stay hand-written.

### What the compile test changed about the design

Handles were originally going to be a cast:

```c
auto* d = static_cast<struct GpioDescriptor*>(check_handle(state, 1, "GpioDescriptor"));
```

That is wrong. `DeviceHandle` in `device.cpp` is not a bare pointer - it carries a
`released` flag so an explicit `release()` followed by collection does not double-put the
kernel's reference, and the accessor is what raises when a script uses a handle it already
gave up. **No signature can express that policy.** So generated code calls a hand-written
accessor instead:

```c
auto* argument0 = lua_bindings_check_gpio_descriptor(state, 1);
```

Generation stops at the boundary and defers to the code that owns the lifetime rules. That
is the pattern for every handle type, and it is the clearest illustration of where this
approach ends.
