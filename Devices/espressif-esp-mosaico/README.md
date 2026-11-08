# Espressif ESP-Mosaico

Tactility device definition for the ESP32-S31 based ESP-Mosaico.

This definition currently targets **board revision v1.2**, which is the revision
seen shipping. Revision v1.0 uses a different pin map:

| Function | v1.2 | v1.0 |
| --- | --- | --- |
| Main I2C SDA / SCL | GPIO56 / GPIO3 | GPIO0 / GPIO1 |
| LCD SCL / RST | GPIO42 / GPIO44 | GPIO44 / GPIO42 |
| Speaker codec rail | always on | GPIO56, active-high |

Supporting both revisions in one image requires reading the board version from
eFuse `USER_DATA` and building the variant-dependent devices at runtime.

## Hardware

- ESP32-S31, 16 MB flash, 250 MHz OCT PSRAM
- 2.16" 480x480 CO5300 AMOLED over QSPI, rounded corners
- CST9217/CST9220 touch over I2C
- ES8311 audio codec + speaker amplifier enable on GPIO45
- BMI270 IMU at 0x69
- BQ27220 fuel gauge at 0x55
- AI button on GPIO7 (deep-sleep wake source only; no keyboard driver)
- Vibration motor on GPIO8 (LEDC)
- USB OTG / TinyUSB mass storage
- SPI NAND (GD5F1GM7, 128 MB) on SPI3, used as the FAT user-data volume (`/sdcard`)

## Display

The CO5300 QSPI bus uses:

- CS 50, SCL 42, D0 36, D1 51, D2 35, D3 9, RST 44
- TE 43 (frame-sync input, used to align each transfer to a panel frame)
- Panel brightness via command 0x51 (no backlight GPIO)

The devicetree supplies the vendor init sequence and the rounded-corner radius;
the CO5300 driver itself contains no board-specific bring-up data.

## Power

GPIO60 is the active-low VCC_3V3 enable. It is driven low by a `gpio-hog` before
any peripheral starts. The official BSP ramps this rail over 50 ms to limit
inrush; this definition currently switches it directly. If brownouts appear,
replace the hog with a board power driver that uses LEDC soft-start.

GPIO57 is the board shutdown signal: an open-drain request line to the SAM8108 power
controller, externally pulled up and only ever driven low. It is driven by
`Drivers/gpio-poweroff-module` (compatible `gpio-power-off`), which keeps the pin
high-impedance while idle and switches it to a low output when something calls
`power_supply_power_off()` (Power app → "Power off"). The line stays asserted for
`assert-timeout-ms` (default 8 s) and is released again if power does not actually drop.

## Storage

The 1 Gbit SPI NAND (GD5F1GM7) is the user-data volume, not just a cache:

- `Drivers/spi-nand-module` (compatible `espressif,spi-nand`) initializes SPI3 with
  CLK 20, DI 21, DO 22, CS 23, HOLD 24, WP 25. HOLD#/WP# are driven high as GPIOs.
- `espressif/spi_nand_flash` provides Dhara wear leveling and bad-block handling. A local
  fork in `Libraries/espressif__spi_nand_flash` adds the ESP32-S31 short-command polling fix
  from the official BSP; without it the interrupt/DMA path stalls on zero-length PAGE READ
  and GET FEATURE (STATUS) transactions, so READY polling hangs.
- `espressif/spi_nand_flash_fatfs` puts FAT on top. Its diskio maps one FAT sector to one
  NAND page (2048 B), so this device raises `CONFIG_WL_SECTOR_SIZE` to 4096 to make
  `FF_MAX_SS` large enough. The NOR wear-levelling `data` partition is unused in SD mode,
  so that change is safe here.
- The volume mounts at `/sdcard` as an SD-card-type device, so Tactility's
  `storage.userDataLocation=SD` puts the whole `tactility/` tree (settings, apps, cache,
  service state) on it. System Info reports 93.2 MiB usable after FTL overhead.
- Capacity breakdown: 128 MiB physical, minus ~8 MiB Dhara journal checkpoints, minus the
  GC reserve (`gc-factor = 4` keeps 1/5 of the FTL capacity free), minus the safety margin
  and FAT metadata. `gc-factor` is a capacity/speed knob: higher values reserve less space
  (~114 MiB at 45, which is what Espressif's BSP uses) but trigger garbage collection much
  more aggressively. 93 MiB is ample for this board, so the default favours write latency
  and lower write amplification (less wear). Changing it requires re-initializing the chip
  with the same value, and lowering it below the formatting value can break the FAT volume.
- FAT is not power-loss safe. Dhara protects its own mapping, but a power cut in the middle
  of a FAT/metadata update can still leave the volume inconsistent. A journaling filesystem
  (LittleFS, as the official BSP uses) would be more robust, at the cost of losing the
  FAT/MSC-style semantics.
- A factory-blank chip is formatted on first mount (`format-if-mount-failed` in the DTS).
  That first boot takes ~22 s; later boots mount in well under a second.
- The partition table switches to the no-data variant, so the NOR `data` partition is gone.

USB drive mode (Settings → USB) exposes this volume to a PC through esp_tinyusb's generic
block-device MSC backend (`tinyusb_msc_new_storage_blockdev`, esp_tinyusb 2.4.0). The local
FAT mount is suspended before the host takes ownership, and the volume is remounted on the
next boot. The host sees the NAND's 2048-byte logical blocks; Windows mounts the FAT volume
successfully. The board's Espressif VID/PID in mass-storage mode is `303A:4002`.

The block device handed to USB is wrapped so writes are committed to Dhara's journal on a
500 ms debounce (and explicitly when leaving drive mode). Without that, a small metadata
update such as a host-side file delete stays in the journal until a later checkpoint: a big
file copy still survived, but deletes reverted on reboot and Windows reported the FAT volume
as needing a scan. The wrapper borrows the real block device; the NAND driver keeps ownership.

## Camera

The left module slot is supported with the V1.4 **SC101IOT** module (8-bit DVP,
UYVY). `Drivers/sc101iot-module` (compatible `smartsens,sc101iot`) initializes
`esp_video`/`esp_cam_sensor` lazily on the first `camera_open()`, center-crops
640x480 to the 480x480 panel, converts UYVY to RGB565 with the PPA, and applies
the DTS rotation/mirror. The built-in Camera app shows the preview, has a
capability-gated flash switch (GPIO34, active-low open-drain), and saves JPEGs
to the app's user-data directory.

The camera module is hinged and can face the user or outward. Like the BSP's
camera app, this is a **manual** Front/Rear toggle, not hinge detection: the
toolbar loop icon flips the preview horizontally (`PPA mirror_x`) and the choice
is persisted in the app's `settings.properties` through Tactility preferences.

Three things are needed besides the driver:

- **DVP stability patch.** IDF 6.1's `esp_driver_cam` does not isolate the tail
  of a short/incomplete frame; without the patch the DMA overruns the capture
  buffer and corrupts memory. The official BSP ships this patch and applies it
  to `$IDF_PATH` at configure time. `Drivers/sc101iot-module/CMakeLists.txt`
  does the same with the 6.1-adapted
  `patches/esp-idf-dvp-frame-capture-stability-6.1.patch`; it is idempotent and
  warns (without failing) if git or a writable IDF checkout is unavailable.
  The 6.1 port also drops frames whose received size is **not exactly** the
  expected size; the app treats those drops as timeouts and keeps streaming.
  This catches short/long frames but not the remaining full-size corruption.
- **AXI GDMA budget.** On S31 the display (SPI2), NAND (SPI3) and camera (CAM0)
  all use the three AXI GDMA pairs. The shared mbedTLS AES/SHA DMA acquires the
  third pair on first use and keeps it, even after Wi-Fi is turned off. `spi0`
  declares `tx-only`, so the SPI driver releases the display's unused RX
  channel after bus init and the DVP camera can use it. Without that,
  `VIDIOC_STREAMON` fails with `no free gdma channel`.
- **Startup settling.** The first frames after `STREAMON` can be partial while
  the DVP FIFO and sensor auto-exposure settle, so the app discards five frames
  before publishing the canvas.

`esp_cam_sensor` does not yet contain the BSP's SC101IOT PCLK-delay fix, so the
driver writes the same register (0x3640 bits [1:0] = 0) over SCCB after
`esp_video_init()`.

**Current status:** the preview is stable with the strict frame-size guard. IDF
6.1's S31 DVP still occasionally produces a full-size but misaligned frame; the
driver drops any frame whose received size is not exactly the expected size, and
the app treats those drops as timeouts so streaming continues. This is a
stability guard rather than a timing fix: 6.2 adds the S31 CAM core clock
source/divider (`cam_ll_select_core_clk_src()` +
`cam_ll_set_core_clock_divider(ctlr, 2, 0, 0)`) and descriptor-node alignment,
which are the proper long-term cleanup when S31 leaves preview. JPEG capture
uses the hardware encoder on the raw UYVY frame, so it is unaffected by the
preview flip. OV3640 (V1.2 module) and the v1.0 board mapping are not
implemented.

## External apps

The board is a full app target:

- `Devices/generic-esp32s31` is the SoC-level device the SDK release pipeline builds (see the
  `Buildscripts` matrix entry in `.github/workflows/build.yml`).
- `Buildscripts/TactilitySDK/sdkconfig.app.esp32s31` is the app-side Kconfig for this target.
- From the apps repository: `tt app bir --architecture esp32s31` builds, installs over the
  dev server (Wi-Fi + Settings → Development), and runs.
- The SDK is installed as `~/.tactility/sdk/<version>/platform-esp32s31/TactilitySDK`; a
  locally generated SDK (via `release-sdk-esp32.py`) or `TACTILITY_SDK_PATH` works the same
  as a downloaded one.
- The SDK's ccache launcher disables IDF's own launcher: nesting them
  (`ccache → cmake -E env → ccache`) breaks on Windows once command lines get long enough
  for ccache to pass an `@response` file.
- Apps must list `esp32s31` in `manifest.properties`' `target.platforms`, or use `all`.

## Flashing

The Type-C port is USB-OTG only; there is no USB-to-UART bridge with DTR/RTS reset
lines. The BOOT button does not reset a running board. It only enters download mode
when held during power-up:

1. Turn the device off.
2. Hold BOOT (GPIO61), power the board on (USB or POWER), then release BOOT.
3. The ROM enumerates its USB-OTG download interface (`303A:0020`) on the same
   Type-C port as a "USB Serial Device".
4. Run `idf.py flash`. This device's sdkconfig selects `--before=no-reset`, so
   esptool connects to the port as-is instead of resetting the board out of
   download mode again. After flashing it hard-resets into the firmware.

While the firmware is running, sending `TT:BOOTDL` on the "Tactility Console" COM
port (`303A:4002`) also reboots into download mode. The console driver sets
`LP_SYSTEM_REG_FORCE_DOWNLOAD_BOOT` (S31's always-on equivalent of
`RTC_CNTL_FORCE_DOWNLOAD_BOOT`) and resets the chip, after which the ROM port from
step 3 appears.

The S31 supports the esptool stub flasher, so this device also sets
`CONFIG_ESPTOOLPY_NO_STUB=n`. The full 4.9 MB image writes in about 30 seconds
instead of the ROM-only path's 2+ minutes.

The same rule applies to the browser flasher: put the board in download mode
manually first.

## Monitoring

The app console is a separate USB device (`303A:4002`), so `idf.py flash monitor`
cannot work in one command. Run:

```
idf.py -p COMx monitor
```

The firmware mirrors logs whenever USB is configured, so the terminal's DTR/RTS
line states do not affect output. `--no-reset` is optional.

The app console only enumerates after the app starts USB, so a fresh power cycle
starts logging around display initialization. ROM, bootloader and early app logs
are not available over this port. Capturing those requires a UART or
USB-Serial/JTAG console on the expansion header.

The monitor's reset shortcut (`Ctrl+T Ctrl+R`) cannot reset this board either,
since the USB-OTG port has no DTR/RTS reset wiring.
