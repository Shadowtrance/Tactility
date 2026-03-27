# usb-host-msc-module

USB Mass Storage Class driver — mounts USB drives into the Tactility filesystem.

## What it does

- Installs the ESP-IDF MSC host driver
- On drive connect: installs the device, mounts it as a FAT filesystem, and registers it with `file_system_add` so it appears in the Files app
- Supports up to 2 simultaneous drives (slot 0 → `/usb`, slot 1 → `/usb1`)
- On drive disconnect: calls `file_system_remove` then cleanly unregisters the VFS and device

## Dependencies

```yaml
dependencies:
  - Drivers/usb-host-module
```

Requires `TactilityKernel`, `usb`, `vfs`, and `fatfs` in CMake.

## sdkconfig requirements

Two entries are needed in `device.properties` (already set for Tab5):

```ini
CONFIG_FATFS_VOLUME_COUNT=6   # default 3 is exhausted by system/data/sdcard
CONFIG_VFS_MAX_COUNT=16       # default 8 fills up before USB drives can register
```

## Notes

- Drives are mounted with `format_if_mount_failed = false` — a drive with an unrecognised or corrupted filesystem is logged and skipped rather than wiped.
- HS USB drives work through hubs. FS/LS devices through a hub are rejected by ESP-IDF's hub driver (`IDF-10023`), but virtually all USB sticks are HS.
