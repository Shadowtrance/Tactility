# luavgl-module

LVGL bindings for Lua, so a script can draw. Separate from `lua-module` so an LVGL version
bump touches one place, and so a runtime can have kernel access without a UI.

Written against **LVGL 9.4**. Inspired by [luavgl](https://github.com/XuNeo/luavgl) (MIT) -
its design informed this, but the code is new. luavgl builds against LVGL 9.1, so the API
gap is small; what did not survive review was its lifetime model (see below).

## Status

Working: the core widget set, the declarative `set{}` dispatcher, widget lifetime
coupling, flex layout, the common style properties, reusable styles, named fonts, the
Tactility toolbar, widget events (including keys), text-area editing and timers.
127 tests, clean under AddressSanitizer.

Widgets: `Object`, `Label`, `Button`, `Slider`, `Switch`, `Checkbox`, `Bar`, `Dropdown`,
`Roller`, `TextArea`, `Line`, `Image`, `Spinner`.

Not done: animations, charts, tables, custom font files.

## Usage

```lua
local screen = lvgl.root()

local panel = lvgl.Object{
    w = lvgl.PCT(100), h = lvgl.SIZE_CONTENT,
    flex_flow = lvgl.FLEX_FLOW.COLUMN,
    bg_color = 0x202020, pad_all = 8, radius = 6,
}

lvgl.Label(panel, { text = "Hello from Lua", text_color = 0xFFFFFF })
lvgl.Label(panel, { text = ("battery %d%%"):format(tactility.power.capacity() or 0) })

local button = lvgl.Button(panel, { w = 120, h = 40 })
lvgl.Label(button, { text = "OK", align = lvgl.ALIGN.CENTER })
```

Constructors take an optional parent and an optional property table, in either order:
`lvgl.Label()`, `lvgl.Label(parent)`, `lvgl.Label{ ... }`, `lvgl.Label(parent, { ... })`.
With no parent a widget attaches to the root the bindings were opened with.

`set{}` returns the widget, so calls chain: `label:set{ text = "x" }:center()`.

### Threading

**Nothing here takes the LVGL lock.** The caller holds it for the lifetime of any script
that uses these bindings, and the script runs on one thread. A script makes many small
LVGL calls, so locking per call would both thrash and leave gaps mid-sequence where
another task could see a half-built UI.

### Geometry is not readable until layout runs

LVGL computes nothing until a layout pass, so this reports 0:

```lua
local label = lvgl.Label{ w = 100 }
local w = label:size()               -- 0
local w = label:update_layout():size()  -- 100
```

LVGL does the pass once per frame; `update_layout()` is for scripts that need the numbers
before then.

## Widgets

```lua
local slider = lvgl.Slider{ w = 200, min = 0, max = 100, value = 50 }
slider:on(lvgl.EVENT.VALUE_CHANGED, function(w)
    tactility.log.info("brightness", w:value())
end)

local toggle = lvgl.Switch{ checked = true }
toggle:is_checked()            -- true
toggle:set_checked(false)

lvgl.Checkbox{ text = "Enable", checked = true }

local menu = lvgl.Dropdown{ options = { "Red", "Green", "Blue" } }
menu:set{ value = 2 }          -- 1-based, like the rest of Lua
menu:value()                   -- 2
menu:text()                    -- "Green"

lvgl.TextArea{ text = "hi", placeholder = "type here", one_line = true }
lvgl.Bar{ min = 0, max = 100, value = 30 }
```

Readers are methods rather than a `get{}` mirror of `set{}`: `slider:value()` reads better,
and it keeps the property table write-only, which is what makes one flat table workable.

- `value()` - slider, bar, dropdown, roller
- `text()` - label, checkbox, text area, and the current selection of a dropdown or roller
- `is_checked()` / `set_checked(bool)` - switch, checkbox
- `cursor()` / `set_cursor(n)` / `insert(text)` / `delete_char()` - text area only

Text area cursors are **1-based**, like the rest of Lua: position `n` means "before
character `n`", so `set_cursor(1)` then `insert(s)` prepends. `delete_char()` removes the
character *before* the cursor, as backspace does.

Each raises on a widget that has no such notion, rather than returning something plausible.

Selections are **1-based** in Lua and 0-based in LVGL; the binding converts, so a script
never sees the difference.

`lvgl.Spinner` is Tactility's spinner from `lvgl-module`, not LVGL's - `LV_USE_SPINNER` is
off in the ESP32 sdkconfig, so `lv_spinner_create()` does not exist there. Widget
availability is per-target and the simulator's `lv_conf.h` is not the firmware's.

## Events

```lua
local button = lvgl.Button{ w = 100, h = 40 }
lvgl.Label(button, { text = "Tap", align = lvgl.ALIGN.CENTER })

button:on(lvgl.EVENT.CLICKED, function(widget)
    tactility.log.info("tapped")
end)

button:off(lvgl.EVENT.CLICKED)   -- stop listening
```

The handler receives its widget, so one function can serve several. Registering twice for
the same event replaces rather than stacking. `on()` returns the widget, so calls chain.

Codes live in `lvgl.EVENT`: `CLICKED`, `SHORT_CLICKED`, `PRESSED`, `RELEASED`,
`LONG_PRESSED`, `VALUE_CHANGED`, `FOCUSED`, `SCROLL`, `KEY` and the rest of the input set.

### Key events

A `KEY` handler receives the key as a second argument:

```lua
editor:on(lvgl.EVENT.KEY, function(widget, key)
    if key == lvgl.KEY.ENTER then
        save()
    elseif key == string.byte('s') then
        -- printable characters arrive as their ASCII value
    end
end)
```

`lvgl.KEY` holds the control codes: `UP`, `DOWN`, `LEFT`, `RIGHT`, `ESC`, `DEL`,
`BACKSPACE`, `ENTER`, `NEXT`, `PREV`, `HOME`, `END`. Printable characters are not in it -
they arrive as ASCII, so compare against `string.byte(...)`.

The key is captured when the event fires, not when the handler runs. Since handlers are
deferred by a cycle (see above), asking the input device later would report whatever key
is current by then - a different keypress, or none.

**Handlers run just after the event, not during it.** They are queued with `lv_async_call`
so a handler may safely delete its own widget - doing that inside LVGL's dispatch would
corrupt the event loop. A widget deleted between the event and the dispatch is skipped.

A handler that raises has nowhere to propagate, since LVGL called it rather than Lua, so
the error is swallowed rather than left to unwind through C.

## Timers

```lua
local timer = lvgl.Timer{
    period = 1000,                       -- ms
    callback = function() refresh() end,
    repeat_count = 1,                    -- optional; 1 makes a one-shot
    paused = false,                      -- optional
}

timer:pause()
timer:resume()
timer:set_period(500)
timer:ready()      -- fire on the next cycle regardless of the period
timer:delete()
```

Collecting the handle does **not** stop the timer - a script that starts one and keeps no
reference still expects it to run. Timers stop when deleted, when their repeat count runs
out, or when the runtime closes.

**Every timer is stopped by `luavgl_bindings_close()`.** This matters more than it looks: a
timer has no widget to anchor it, so nothing else would notice its runtime had gone - it
would simply keep firing into freed memory.

LVGL's `auto_delete` is turned off for these. Left on, LVGL frees an exhausted timer itself
and the Lua handle is left pointing at freed memory; ownership stays with the binding
instead.

## Styles

```lua
local card = lvgl.Style{ bg_color = 0x1c1c28, radius = 6, pad_all = 6 }

for i = 1, 20 do
    lvgl.Object{ w = lvgl.PCT(100), h = 40 }:add_style(card)
end

card:set{ radius = 12 }        -- restyles all twenty
```

Worth having over repeating a `set{}` table because LVGL applies a style by reference:
twenty rows sharing one style cost one style, and restyling them all is one edit.

`add_style` and `remove_style` return the widget, so calls chain. `set{}` on a style
returns the style, the same way.

### Selectors

A third argument narrows a style to one part in one state, which the flat property table
cannot reach:

```lua
slider:add_style(highlight, lvgl.PART.KNOB | lvgl.STATE.PRESSED)
```

Parts: `MAIN`, `SCROLLBAR`, `INDICATOR`, `KNOB`, `SELECTED`, `ITEMS`, `CURSOR`, `ANY`.
States: `DEFAULT`, `CHECKED`, `FOCUSED`, `EDITED`, `HOVERED`, `PRESSED`, `SCROLLED`,
`DISABLED`, `ANY`. `add_style` defaults to the main part in any state; `remove_style`
defaults to every part in every state.

### Style lifetime

The reference LVGL keeps is also the hazard. `lv_obj_add_style()` stores the
`lv_style_t*`; it does not copy. A style freed while a widget still points at it is a
use-after-free on the next redraw, and unlike a widget handle there is no LVGL event to
warn us. So:

- Every style is pinned for the life of the runtime. Collecting the Lua handle does not
  free the underlying style - a script that styles a widget and keeps no reference is
  doing something reasonable.
- `luavgl_bindings_close()` removes every style from every live widget *before* freeing
  them, so a widget outliving the runtime cannot dereference a dead style.

Removal is by pointer rather than clearing the widget, because a widget may also carry
styles a C++ app applied, and those are not the runtime's to strip.

## Fonts

```lua
lvgl.Label{ text = "Title", text_font = 'large' }
lvgl.Style{ text_font = 'small' }
```

`'small'`, `'default'`, `'large'` and `'icon'`. Named rather than raw `lv_font_t*`:
`lvgl-module` picks the face for the display's density, so a script asking for `'large'`
stays legible on a 320x240 panel and a 1280x720 one alike. Exposing pointers would tie a
script to one device.

## Widget lifetime

The hard part, and where this differs most from luavgl.

LVGL owns widgets and deletes children with their parent. Lua's collector knows nothing
about that, so:

- Each handle is registered against its `lv_obj_t*`, so wrapping one widget twice returns
  the same userdata and `==` behaves.
- An `LV_EVENT_DELETE` callback nulls the handle when LVGL frees the widget. A script that
  keeps using it gets `widget has been deleted`, not a use-after-free. This covers the
  case that matters: deleting a parent invalidates handles on every descendant.
- The registry holds handles **weakly**, so a handle the script drops is collectable even
  while the widget lives.
- There is deliberately **no `__gc`** deleting widgets. Collecting a handle must not remove
  something from the screen; a script that wants a widget gone calls `delete()`.

`obj:clean()` deletes every child but keeps the object, which is how a view swap is
normally written - a container is emptied and rebuilt rather than each widget being
tracked and deleted by hand. Handles for the children are invalidated by the same path as
`delete()`, so one held across the clean raises "widget has been deleted" rather than
dangling. This reaches the whole subtree, not just direct children.

### Closing a runtime

```c
luavgl_bindings_close(runtime);   // required
lua_runtime_free(runtime);
```

Widgets routinely outlive the runtime that created them - an app closes its Lua state
while its LVGL tree is still on screen. Each wrapped widget holds a delete callback
pointing at that `lua_State`, so tearing the tree down afterwards would call into freed
memory. `luavgl_bindings_close()` detaches those callbacks; it matches on the state
pointer, so other runtimes sharing the same widgets are unaffected.

## Properties

`set{}` dispatches over a sorted name table rather than exposing one function per LVGL
setter - LVGL has thousands, and this keeps the binding to a few files.

| Group | Names |
| --- | --- |
| Geometry | `w`/`width`, `h`/`height`, `x`, `y`, `align` |
| Text | `text`, `long_mode` |
| Value | `value`, `min`, `max`, `options`, `checked` |
| Text area | `placeholder`, `one_line`, `password_mode` |
| Image | `src` |
| Style | `bg_color`, `bg_opa`, `text_color`, `text_font`, `border_color`, `border_width`, `radius`, `pad_all`, `opa` |
| Layout | `flex_flow`, `flex_grow`, `pad_row`, `pad_column` |

`pad_row`/`pad_column` are the gap *between* flex children, which `pad_all` does not touch.
Without them a Lua screen looks visibly looser than a C++ one built the same way.

Colours are `0xRRGGBB` integers. An unknown name raises rather than being ignored, so a
typo fails loudly. `text` checks the widget class first, since LVGL's own setters do not
and would otherwise write through the wrong widget.

Constants live in `lvgl.ALIGN`, `lvgl.FLEX_FLOW`, `lvgl.LONG`, `lvgl.FLAG`, `lvgl.OPA`,
`lvgl.PART` and `lvgl.STATE`. `lvgl.PCT(n)` and `lvgl.SIZE_CONTENT` cover sizing.
