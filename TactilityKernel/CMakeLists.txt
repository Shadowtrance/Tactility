cmake_minimum_required(VERSION 3.20)

include("${CMAKE_CURRENT_LIST_DIR}/../Buildscripts/module.cmake")

file(GLOB_RECURSE SOURCE_FILES "source/*.c**")

list(APPEND REQUIRES_LIST)

if (DEFINED ENV{ESP_IDF_VERSION})
    # TODO move the related logic for esp_time in Tactility/time.h into the Platform/ subproject
    list(APPEND REQUIRES_LIST esp_timer)
    # elf_run.c wraps the ELF loader so that a loaded app can run further ELF images of its own.
    list(APPEND REQUIRES_LIST elf_loader)
else ()
    list(APPEND REQUIRES_LIST freertos_kernel)
    # No ELF loader off-target: elf_run.c is ESP-only.
    list(REMOVE_ITEM SOURCE_FILES "${CMAKE_CURRENT_LIST_DIR}/source/elf_run.c")
endif ()

tactility_add_module(TactilityKernel
    SRCS ${SOURCE_FILES}
    INCLUDE_DIRS include/
    PRIV_INCLUDE_DIRS private/
    REQUIRES ${REQUIRES_LIST}
)
