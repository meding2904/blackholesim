cmake_minimum_required(VERSION 3.24)
project(BlackHoleSimulation LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---------------------------------------------------------------------------
# raylib (pinned, current stable release)
# ---------------------------------------------------------------------------
set(RAYLIB_VERSION 6.2)
set(RAYLIB_VERIFY_VERSION ON CACHE BOOL "Verify raylib version at configure time")

find_package(raylib ${RAYLIB_VERSION} QUIET)
if(NOT raylib_FOUND)
    message(STATUS "raylib ${RAYLIB_VERSION} not found locally, fetching source")
    include(FetchContent)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        raylib
        URL https://github.com/raysan5/raylib/archive/refs/tags/${RAYLIB_VERSION}.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP OFF
    )
    FetchContent_MakeAvailable(raylib)
endif()

# ---------------------------------------------------------------------------
# Application
# ---------------------------------------------------------------------------
add_library(blackhole_physics INTERFACE)
target_include_directories(blackhole_physics INTERFACE include)

add_executable(blackhole
    src/main.cpp
    src/application.cpp
    src/camera_controller.cpp
    src/renderer.cpp
    src/simulation_parameters.cpp
    src/textures.cpp
    src/ui.cpp
)
target_include_directories(blackhole PRIVATE include)
target_link_libraries(blackhole PRIVATE raylib blackhole_physics)
target_compile_options(blackhole PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
)
target_compile_definitions(blackhole PRIVATE
    BLACKHOLE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

if(WIN32)
    target_compile_definitions(blackhole PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# Shaders and runtime assets live next to the executable so the app runs from
# the build directory and from an install tree without absolute paths.
add_custom_command(TARGET blackhole POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders"
        "$<TARGET_FILE_DIR:blackhole>/shaders"
    COMMENT "Copying shaders next to the executable"
)

# ---------------------------------------------------------------------------
# Physics tests (no GPU, no window required)
# ---------------------------------------------------------------------------
add_executable(physics_tests tests/physics_tests.cpp src/simulation_parameters.cpp)
target_link_libraries(physics_tests PRIVATE blackhole_physics)

target_compile_options(physics_tests PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
)

enable_testing()
add_test(NAME physics_tests COMMAND physics_tests)
