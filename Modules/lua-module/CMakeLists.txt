cmake_minimum_required(VERSION 3.20)

include("${CMAKE_CURRENT_LIST_DIR}/../../Buildscripts/module.cmake")

file(GLOB_RECURSE SOURCE_FILES "source/*.c*")

list(APPEND REQUIRES_LIST
    TactilityKernel
    lua
)

tactility_add_module(lua-module
    SRCS ${SOURCE_FILES}
    INCLUDE_DIRS include/
    PRIV_INCLUDE_DIRS private/ source/bindings/
    REQUIRES ${REQUIRES_LIST}
)
