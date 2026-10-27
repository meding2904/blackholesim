cmake_minimum_required(VERSION 3.24)
project(BlackHoleSimulation VERSION 1.0.0 LANGUAGES C CXX)

include(CTest)
include(GNUInstallDirs)

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
    # GitHub archive tags are pinned by content hash. If GitHub ever regenerates
    # the archive and the hash check fails, download the tarball and refresh it:
    #   curl -L -o raylib.tar.gz \
    #     https://github.com/raysan5/raylib/archive/refs/tags/6.2.tar.gz
    #   sha256sum raylib.tar.gz
    FetchContent_Declare(
        raylib
        URL https://github.com/raysan5/raylib/archive/refs/tags/${RAYLIB_VERSION}.tar.gz
        URL_HASH SHA256=df7a13129b1bec9e73a6962efff170beba310ee6eab8f94cd000b7316b4db5ee
        DOWNLOAD_EXTRACT_TIMESTAMP OFF
    )
    FetchContent_MakeAvailable(raylib)
endif()

# raylib's own CMake project does not declare a project VERSION, so fall back
# to its public header when raylib_VERSION is unavailable (e.g. FetchContent).
if(RAYLIB_VERIFY_VERSION)
    set(_raylib_version "${raylib_VERSION}")
    if(NOT _raylib_version AND DEFINED raylib_SOURCE_DIR AND
       EXISTS "${raylib_SOURCE_DIR}/src/raylib.h")
        file(STRINGS "${raylib_SOURCE_DIR}/src/raylib.h" _raylib_version_line
             REGEX "^#define RAYLIB_VERSION +\"")
        string(REGEX REPLACE "^.*\"([0-9]+\\.[0-9]+).*$" "\\1"
               _raylib_version "${_raylib_version_line}")
    endif()

    if(NOT _raylib_version)
        message(WARNING "Could not determine the raylib version; "
                        "RAYLIB_VERIFY_VERSION check skipped.")
    elseif(_raylib_version VERSION_LESS RAYLIB_VERSION)
        message(FATAL_ERROR
            "raylib ${_raylib_version} is too old: ${RAYLIB_VERSION} or newer is required.")
    else()
        message(STATUS "Using raylib ${_raylib_version} (required >= ${RAYLIB_VERSION})")
    endif()
endif()

# ---------------------------------------------------------------------------
# Application
# ---------------------------------------------------------------------------
add_library(blackhole_physics INTERFACE)
target_include_directories(blackhole_physics INTERFACE include)
target_compile_features(blackhole_physics INTERFACE cxx_std_20)

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

set(BH_WARNING_FLAGS
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4;/permissive->
)
target_compile_options(blackhole PRIVATE ${BH_WARNING_FLAGS})
target_compile_definitions(blackhole PRIVATE
    BLACKHOLE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

if(WIN32)
    target_compile_definitions(blackhole PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# Shaders and runtime assets live next to the executable so the app runs from
# the build directory and from an install tree without absolute paths. The
# stamp makes the copy dependency-tracked: it only runs when a shader changes
# (or when the target directory is new), not on every build.
file(GLOB SHADER_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/shaders/*.fs")
set(SHADER_STAMP "${CMAKE_CURRENT_BINARY_DIR}/shaders-$<CONFIG>.stamp")
add_custom_command(
    OUTPUT "${SHADER_STAMP}"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders"
        "$<TARGET_FILE_DIR:blackhole>/shaders"
    COMMAND ${CMAKE_COMMAND} -E touch "${SHADER_STAMP}"
    DEPENDS ${SHADER_FILES}
    COMMENT "Copying shaders next to the executable"
    VERBATIM
)
add_custom_target(copy_shaders ALL DEPENDS "${SHADER_STAMP}")
add_dependencies(blackhole copy_shaders)

# ---------------------------------------------------------------------------
# Physics tests (no GPU, no window required)
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
    add_executable(physics_tests tests/physics_tests.cpp src/simulation_parameters.cpp)
    target_link_libraries(physics_tests PRIVATE blackhole_physics)
    target_compile_options(physics_tests PRIVATE ${BH_WARNING_FLAGS})

    add_test(NAME physics_tests COMMAND physics_tests)
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS blackhole RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
# resolveAssetPath() looks for "shaders/" next to the executable.
install(DIRECTORY shaders/ DESTINATION ${CMAKE_INSTALL_BINDIR}/shaders)
install(FILES LICENSE README.md DESTINATION ${CMAKE_INSTALL_DATADIR}/blackhole)
