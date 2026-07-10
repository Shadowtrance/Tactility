file(GLOB_RECURSE SOURCE_FILES Source/*.c*)

idf_component_register(
    SRCS ${SOURCE_FILES}
    INCLUDE_DIRS "Source"
    REQUIRES Tactility EspLcdCompat ST7789 GT911 PwmBacklight EstimatedPower driver audio-codec-module es7210-module dummy-i2s-amp-module audio-stream-module
)
