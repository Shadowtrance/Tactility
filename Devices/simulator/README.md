# Simulator

Runs Tactility on the host via SDL, for development without hardware.

Linux only - this does **not** build on native Windows. WSL2 works.

```bash
cmake -B buildsim -G Ninja
ninja -C buildsim
./buildsim/Firmware/FirmwareSim
```

## Display size

`TACTILITY_DISPLAY` takes a preset name or an explicit `<width>x<height>`:

```bash
TACTILITY_DISPLAY=cardputer ./buildsim/Firmware/FirmwareSim   # 240x135
TACTILITY_DISPLAY=cyd       ./buildsim/Firmware/FirmwareSim   # 240x320, portrait
TACTILITY_DISPLAY=tdeck     ./buildsim/Firmware/FirmwareSim   # 320x240 (the default)
TACTILITY_DISPLAY=tab5      ./buildsim/Firmware/FirmwareSim   # 1280x720
TACTILITY_DISPLAY=480x320   ./buildsim/Firmware/FirmwareSim
```

| Preset | Size | Devices |
| --- | --- | --- |
| `cardputer` | 240x135 | M5Stack Cardputer |
| `cyd` | 240x320 | CYD boards, Elecrow CrowPanel (portrait) |
| `tdeck` | 320x240 | LilyGO T-Deck, M5Stack CoreS3 |
| `tab5` | 1280x720 | M5Stack Tab5 |

An unusable value logs a warning and falls back to the default, rather than refusing to
start.

### Fonts and UI density

These come from `Devices/simulator/device.properties`, not from the environment, so they
are **build**-time rather than runtime:

```properties
lvgl.fontSize=14      # 10/14/18, icons 16/36/16
lvgl.uiDensity=default
```

`lvgl.fontSize` picks a tier of six sizes (small/default/large plus three icon sizes) using
the same table `device.py` applies to real devices - see `Modules/lvgl-module/CMakeLists.txt`.
Matching a specific board therefore means copying its `lvgl.fontSize` here and rebuilding:

```bash
# Devices/simulator/device.properties: lvgl.fontSize=28 (as on m5stack-tab5)
ninja -C buildsim
TACTILITY_DISPLAY=tab5 ./buildsim/Firmware/FirmwareSim
```

Without that, `TACTILITY_DISPLAY` still changes the canvas, which is what matters for
checking layout, overflow and clipping - just not the type size.

`lv_conf.h` enables Montserrat 8 through 48 so every tier links. That file is used by the
simulator only; ESP32 builds take their LVGL config from `sdkconfig` via the managed
component, so this costs no firmware flash.

## Apps

Lua apps run here; ELF apps do not, since they need a loader rather than an interpreter
(`AppInstance.h` falls back to `check(false, "not supported")` off-target).

Install by opening a `.app` in the Files app and confirming the prompt. Apps are installed
under `data/`, which the simulator registers as a file system at boot and scans on startup.

## Renderer

The SDL software renderer is used by default. The accelerated backend renders nothing at
all under WSLg - every call succeeds and the window stays black - and the failure cannot be
detected at runtime, only observed. Set `TACTILITY_SDL_ACCELERATED=1` to opt back in on a
native Linux desktop with a working GPU driver.

```
I (44) SdlDisplay SDL video driver='x11' renderer='software'
```

That line is logged at startup; it is the first thing to check if the window is blank.
