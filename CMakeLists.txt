cmake_minimum_required(VERSION 3.10)
project(osr)

if (NOT DEFINED CMAKE_MSVC_RUNTIME_LIBRARY)
    if (OSR_MIMALLOC)
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    else ()
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    endif ()
endif ()

include(cmake/buildcache.cmake)
include(cmake/pkg.cmake)

# --- LINT ---
option(OSR_LINT "Run clang-tidy with the compiler." OFF)
if (OSR_LINT)
    # clang-tidy will be run on all targets defined hereafter
    include(cmake/clang-tidy.cmake)
endif ()

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(osr-compile-options
            -Weverything
            -Wno-c++98-compat
            -Wno-c++98-compat-pedantic
            -Wno-newline-eof
            -Wno-missing-prototypes
            -Wno-padded
            -Wno-double-promotion
            -Wno-undef
            -Wno-undefined-reinterpret-cast
            -Wno-float-conversion
            -Wno-global-constructors
            -Wno-exit-time-destructors
            -Wno-switch-enum
            -Wno-c99-designator
            -Wno-zero-as-null-pointer-constant
            -Wno-missing-noreturn
            -Wno-undefined-func-template
            -Wno-unsafe-buffer-usage
            -Wno-c++20-compat
            -Wno-reserved-macro-identifier
            -Wno-documentation-unknown-command
            -Wno-duplicate-enum
            -Werror)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
    set(osr-compile-options -Wall -Wextra -Werror)
elseif (MSVC)
    set(osr-compile-options /WX)
else ()
    set(osr-compile-options
            -Wall
            -Wextra
            -Wno-maybe-uninitialized)
    if (NOT CMAKE_CROSSCOMPILING)
        set(osr-compile-options ${osr-compile-options} -Werror)
    endif ()
endif ()

# --- LIB ---
file(GLOB_RECURSE osr-src src/*.cc)
add_library(osr ${osr-src})
target_include_directories(osr PUBLIC include)
target_compile_features(osr PUBLIC cxx_std_23)
target_compile_options(osr PRIVATE ${osr-compile-options})
target_link_libraries(osr
        osmium
        geo
        cista
        utl
        protozero
        tiles-import-library
        rtree
        tg
        lmdb
        unordered_dense
        boost-thread
        expat
        rapidjson
)

# --- MAIN ---
add_executable(osr-extract exe/extract.cc)
target_link_libraries(osr-extract osr)

# --- TEST ---
configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_dir.h.in
        ${CMAKE_CURRENT_BINARY_DIR}/generated/test_dir.h
)
file(GLOB_RECURSE osr-test-files test/*cc)
add_executable(osr-test ${osr-test-files})
target_include_directories(osr-test PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_link_libraries(osr-test gtest osr)