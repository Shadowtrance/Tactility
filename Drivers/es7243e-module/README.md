# ES7243E microphone ADC

A driver for the `ES7243E` stereo audio ADC by Everest Semiconductor, wired as an
input-only `AUDIO_CODEC_TYPE` device. The I2C bus is the device's parent, the I2S
controller carrying the audio data is referenced via a devicetree phandle. The ADC
runs as I2S slave with MCLK from the pad (MCLK = 256 x sample rate).

Wraps Espressif's `esp_codec_dev` ES7243E implementation.

License: [Apache v2.0](LICENSE-Apache-2.0.md)
