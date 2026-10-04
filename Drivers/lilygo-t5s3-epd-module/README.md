# lilygo-t5s3-epd-module

A kernel driver for the 4.7" 960x540 `ED047TC1` e-paper panel of the LILYGO T5 4.7 Inch E-Paper S3 (V2.3/V2.4).

The driver only works on this board. The pins of the panel, the 74HCT4094 shift register that carries the panel supplies and the sequence of the panel are fixed in the code, so the devicetree node only has the pixel clock as property.

- `t5s3_display.cpp`: the Tactility display driver. It decides between fast and quality updates.
- `t5s3_epd.cpp`: the engine. It controls the shift register, the CKV gate clock (RMT peripheral), the row transfer (LCD peripheral in i80 mode), the update loop and the clear sequence.
- `t5s3_epd_rows.h`: the conversion of framebuffer rows into bus data. It has no hardware access.
- `t5s3_epd_waveform.h`: the 30 phase drive table of the quality updates.

There are three update modes. Fast draws black and white only. Quality draws 16 levels and leaves white pixels that stay white alone, which is used for partial updates. Full also flashes those white pixels, which is used for the clear and the refresh.

## License

The engine and its tables are derived from [epdiy](https://github.com/vroland/epdiy) and are licensed under the
[LGPL v3.0 or later](LICENSE-LGPL-3.0.md) (the LGPL supplements the [GPL v3.0](../../Documentation/LICENSE-GPL-3.0.md)):

- `source/t5s3_epd.cpp`
- `source/t5s3_epd.h`
- `source/t5s3_epd_rows.h`
- `source/t5s3_epd_waveform.h`

These files follow the sequences and timings of epdiy and contain its ED047TC1 waveform data. Each of them has an `SPDX-License-Identifier: LGPL-3.0-or-later` header.

The rest of the module (`source/t5s3_display.cpp`, `source/module.cpp`, the bindings and the headers in `include/`) is licensed under [Apache License v2.0](LICENSE-Apache-2.0.md).

A project that contains this module has to follow the LGPL for the four files above.
