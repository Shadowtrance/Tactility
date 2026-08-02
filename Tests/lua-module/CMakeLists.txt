project(LuaModuleTests)

enable_language(C CXX ASM)

file(GLOB_RECURSE TEST_SOURCES ${PROJECT_SOURCE_DIR}/Source/*.cpp)
add_executable(LuaModuleTests EXCLUDE_FROM_ALL ${TEST_SOURCES})

target_include_directories(LuaModuleTests PRIVATE ${DOCTESTINC})

add_test(NAME LuaModuleTests COMMAND LuaModuleTests)

target_link_libraries(LuaModuleTests PUBLIC
    lua-module
    lua # ModuleTest.cpp checks the exported symbols against the real Lua functions
    TactilityKernel
    freertos_kernel
)
