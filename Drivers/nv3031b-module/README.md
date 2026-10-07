# NV3031B display

A driver for the `NV3031B` display panel by NewVision, over QSPI.

The esp_lcd panel (`esp_lcd_nv3031b.c`) is adapted from Espressif's
[esp_lcd_sh8601](https://github.com/espressif/esp-iot-solution/tree/master/components/display/lcd/esp_lcd_sh8601),
which uses the same QSPI command framing.

License:

- [Apache v2.0](LICENSE-Apache-2.0.md) for all files except the one below
- [BSD 2-Clause](LICENSE-BSD-2-Clause.md) for `source/nv3031b_init_cmds.h`, the initialization sequence from [LovyanGFX](https://github.com/lovyan03/LovyanGFX)
