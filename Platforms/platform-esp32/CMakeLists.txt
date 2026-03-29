cmake_minimum_required(VERSION 3.20)

file(GLOB_RECURSE SOURCES "source/*.c**")

set(EXTRA_REQUIRES "")
if(CONFIG_BT_NIMBLE_ENABLED)
    list(APPEND EXTRA_REQUIRES bt)
endif()

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "include/"
    PRIV_INCLUDE_DIRS "private/"
    REQUIRES TactilityKernel driver vfs fatfs ${EXTRA_REQUIRES}
)
