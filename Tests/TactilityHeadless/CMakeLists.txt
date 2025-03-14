project(TactilityCoreTests)

enable_language(C CXX ASM)

set(CMAKE_CXX_COMPILER g++)

file(GLOB_RECURSE TEST_SOURCES ${PROJECT_SOURCE_DIR}/*.cpp)
add_executable(TactilityHeadlessTests EXCLUDE_FROM_ALL ${TEST_SOURCES})

add_definitions(-D_Nullable=)
add_definitions(-D_Nonnull=)

target_include_directories(TactilityHeadlessTests PRIVATE
    ${DOCTESTINC}
)

add_test(NAME TactilityHeadlessTests
    COMMAND TactilityHeadlessTests
)

target_link_libraries(TactilityHeadlessTests PRIVATE
    Tactility
    TactilityCore
    TactilityHeadless
    Simulator
    SDL2::SDL2-static SDL2-static
)
