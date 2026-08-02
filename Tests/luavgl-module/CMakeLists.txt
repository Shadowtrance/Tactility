project(LuavglModuleTests)

enable_language(C CXX ASM)

file(GLOB_RECURSE TEST_SOURCES ${PROJECT_SOURCE_DIR}/Source/*.cpp)
add_executable(LuavglModuleTests EXCLUDE_FROM_ALL ${TEST_SOURCES})

target_include_directories(LuavglModuleTests PRIVATE
    ${DOCTESTINC}
    # The event callbacks live on internal widgets the bindings do not return, so the
    # detach test needs the module's private header to inspect them.
    ${PROJECT_SOURCE_DIR}/../../Modules/luavgl-module/private
)

add_test(NAME LuavglModuleTests COMMAND LuavglModuleTests)

target_link_libraries(LuavglModuleTests PUBLIC
    luavgl-module
    lua-module
    lua
    lvgl-module # the toolbar bindings wrap its widget
    lvgl
    TactilityKernel
    freertos_kernel
)
