cmake_minimum_required(VERSION 3.20)

include("${CMAKE_CURRENT_LIST_DIR}/../../Buildscripts/module.cmake")

file(GLOB_RECURSE SOURCE_FILES "source/*.c*")

tactility_add_module(sc101iot-module
    SRCS ${SOURCE_FILES}
    INCLUDE_DIRS include/
    REQUIRES TactilityKernel platform-esp32 esp_video esp_cam_sensor driver esp_new_jpeg esp_driver_ppa esp_mm
)

# ESP-IDF 6.1's DVP driver does not isolate the bytes of a short/incomplete frame. The DMA then
# writes past the capture buffer and corrupts memory (observed as a PSRAM load-access fault).
# The official ESP-Mosaico BSP ships this stability patch and applies it to IDF at configure time;
# keep the same approach here so the fix travels with the camera driver. The operation is
# idempotent and only a warning if it cannot be applied.
set(SC101IOT_DVP_PATCH "${CMAKE_CURRENT_LIST_DIR}/patches/esp-idf-dvp-frame-capture-stability-6.1.patch")
find_program(SC101IOT_GIT git)

function(sc101iot_apply_idf_dvp_patch)
    if(NOT SC101IOT_GIT)
        message(WARNING "sc101iot-module: Git not found; the ESP-IDF DVP stability patch was not applied and camera capture may be unstable")
        return()
    endif()
    if(NOT DEFINED ENV{IDF_PATH} OR "$ENV{IDF_PATH}" STREQUAL "")
        message(WARNING "sc101iot-module: IDF_PATH is unavailable; the ESP-IDF DVP stability patch was not applied and camera capture may be unstable")
        return()
    endif()

    execute_process(
        COMMAND "${SC101IOT_GIT}" -C "$ENV{IDF_PATH}" apply --no-index --reverse --check "${SC101IOT_DVP_PATCH}"
        RESULT_VARIABLE SC101IOT_PATCH_APPLIED
        OUTPUT_QUIET
        ERROR_QUIET
    )
    if(SC101IOT_PATCH_APPLIED STREQUAL "0")
        message(STATUS "sc101iot-module: ESP-IDF DVP stability patch already applied")
        return()
    endif()

    execute_process(
        COMMAND "${SC101IOT_GIT}" -C "$ENV{IDF_PATH}" apply --no-index "${SC101IOT_DVP_PATCH}"
        RESULT_VARIABLE SC101IOT_PATCH_RESULT
        ERROR_VARIABLE SC101IOT_PATCH_ERROR
        ERROR_STRIP_TRAILING_WHITESPACE
    )
    if(SC101IOT_PATCH_RESULT STREQUAL "0")
        message(STATUS "sc101iot-module: applied ESP-IDF DVP stability patch")
    else()
        message(WARNING "sc101iot-module: failed to apply the ESP-IDF DVP stability patch; camera capture may crash on short frames: ${SC101IOT_PATCH_ERROR}")
    endif()
endfunction()

sc101iot_apply_idf_dvp_patch()
