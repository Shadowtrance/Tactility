cmake_minimum_required(VERSION 3.20)

# Lua's internal headers live in Source/ next to the .c files that include them, so
# Source/ is a private include dir on both targets. Only Include/ is public.

if (DEFINED ENV{ESP_IDF_VERSION})
    idf_component_register(
        SRC_DIRS "Source/"
        INCLUDE_DIRS "Include/"
        PRIV_INCLUDE_DIRS "Source/"
    )

    # Upstream Lua is warning-clean under its own Makefile but not under IDF's stricter
    # default set. These are in third-party code we do not want to patch.
    target_compile_options(${COMPONENT_LIB} PRIVATE
        -Wno-implicit-fallthrough
        -Wno-type-limits
    )

else()
    file(GLOB SOURCES "Source/*.c")

    add_library(lua STATIC)

    target_sources(lua PRIVATE ${SOURCES})
    target_include_directories(lua
        PUBLIC "Include/"
        PRIVATE "Source/"
    )

    # Enables the POSIX bits of the standard library (io.popen, os.execute) that the
    # simulator can genuinely support.
    target_compile_definitions(lua PRIVATE LUA_USE_LINUX)
    target_link_libraries(lua PRIVATE m dl)

    target_compile_options(lua PRIVATE
        -Wno-implicit-fallthrough
        -Wno-type-limits
    )

endif()
