# usb-host-module

Core USB host lifecycle driver for ESP32-P4 (and any SoC with `SOC_USB_OTG_SUPPORTED`).

## What it does

- Installs the ESP-IDF USB host library (`usb_host_install`)
- Runs the `usb_lib` background task that dispatches USB host library events (device attach/detach, free-all, etc.)
- Exposes `usb_host_is_running()` so other modules can gate on host readiness

## Dependencies

None beyond `TactilityKernel` and the ESP-IDF `usb` component.

## Usage

Add to your device's `devicetree.yaml`:

```yaml
dependencies:
  - Drivers/usb-host-module
```

This module must be listed **before** any class-driver modules (`usb-host-hid-module`, `usb-host-msc-module`) so the host library is installed first.

## Notes

- `skip_phy_setup = false` — lets ESP-IDF configure the HS OTG PHY automatically. Correct for ESP32-P4.
- The `usb_lib` task runs at priority 10 on any core.
