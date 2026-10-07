# ADS1115 ADC

A driver for the Texas Instruments `ADS1115` 16-bit I2C ADC, exposed as an ADC controller
with the four single-ended inputs AIN0 to AIN3 as channels 0 to 3.

Each read is a single-shot conversion. The result is returned as a 12-bit value: 0 to 4095
covers 0 V to the configured full-scale voltage, negative readings are returned as 0.
Written from the TI datasheet (SBAS444).

License: [Apache v2.0](LICENSE-Apache-2.0.md)
