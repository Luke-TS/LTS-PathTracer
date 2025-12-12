cmake_minimum_required(VERSION 3.18)
project(ray_tracer LANGUAGES CXX CUDA VERSION 1.0.0)

# ═════════════════════════════════════
#  Build Options
# ═════════════════════════════════════
option(BUILD_TESTS "Build unit tests" OFF)
option(ENABLE_CUDA "Enable CUDA acceleration" OFF)

# ═════════════════════════════════════
#  Compiler Settings
# ═════════════════════════════════════
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(ENABLE_CUDA)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)
    set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
    set(CMAKE_CUDA_ARCHITECTURES native CACHE STRING "GPU architectures")
endif()

# Always export compile_commands.json for clangd/LSP
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ═════════════════════════════════════
#  Output Directories (cleaner build tree)
# ═════════════════════════════════════
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# ═════════════════════════════════════
#  Source Organization (explicit is better than GLOB_RECURSE)
# ═════════════════════════════════════

# Core utilities
set(CORE_SOURCES
    src/core/types/interval.cpp
)

# Geometry
set(GEOMETRY_SOURCES
    src/geometry/primitives/sphere.cpp
    src/geometry/primitives/triangle.cpp
    src/geometry/primitives/quad.cpp
    src/geometry/primitives/rect.cpp
    src/geometry/composite/cuboid.cpp
    src/geometry/composite/instance.cpp
    src/geometry/accel/aabb.cpp
    src/geometry/accel/bvh.cpp
)

# Materials & Textures
set(MATERIAL_SOURCES
    src/materials/material.cpp
)

set(TEXTURE_SOURCES
    src/textures/texture.cpp
)

# Scene
set(SCENE_SOURCES
    src/scene/scene.cpp
    src/scene/scene_loader.cpp
    src/scene/camera.cpp
    src/scene/texture_image.cpp
    src/scene/cubemap_utils.cpp
)

# Rendering
set(RENDERER_SOURCES
    src/rendering/framebuffer.cpp
    src/rendering/wavefront.cpp
)

# Third-party (if you have .cpp implementations)
set(THIRD_PARTY_SOURCES
    # Add any .cpp files from third-party if needed
)

# CUDA sources (only if CUDA is enabled)
if(ENABLE_CUDA)
    set(CUDA_SOURCES
        # Add .cu files here as you create them
        # src/rendering/integrators/gpu_ray_integrator.cu
    )
endif()

# ═════════════════════════════════════
#  Main Executable
# ═════════════════════════════════════
add_executable(${PROJECT_NAME}
    src/main.cpp
    ${CORE_SOURCES}
    ${GEOMETRY_SOURCES}
    ${MATERIAL_SOURCES}
    ${TEXTURE_SOURCES}
    ${SCENE_SOURCES}
    ${RENDERER_SOURCES}
    ${THIRD_PARTY_SOURCES}
)

if(ENABLE_CUDA AND CUDA_SOURCES)
    target_sources(${PROJECT_NAME} PRIVATE ${CUDA_SOURCES})
endif()

# ═════════════════════════════════════
#  Include Directories
# ═════════════════════════════════════
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src  # So you can #include "core/vec3.h"
    SYSTEM PRIVATE
        ${CMAKE_SOURCE_DIR}/third-party  # Third-party headers
)

# ═════════════════════════════════════
#  Dependencies
# ═════════════════════════════════════

# TinyObjLoader (local external)
if(EXISTS ${CMAKE_SOURCE_DIR}/external/tinyobjloader)
    set(TINYOBJLOADER_BUILD_TEST_LOADER OFF)
    set(TINYOBJLOADER_BUILD_OBJ_STICHER OFF)
    add_subdirectory(external/tinyobjloader EXCLUDE_FROM_ALL)
    target_link_libraries(${PROJECT_NAME} PRIVATE tinyobjloader)
endif()

# nlohmann/json (FetchContent)
include(FetchContent)
FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(json)

# OpenMP (optional but recommended)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAS_OPENMP)
endif()

# Link all dependencies
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        nlohmann_json::nlohmann_json
)

# ═════════════════════════════════════
#  Compiler Warnings
# ═════════════════════════════════════
target_compile_options(${PROJECT_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:
        $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Wpedantic>
        $<$<CXX_COMPILER_ID:MSVC>:/W4>
    >
    $<$<COMPILE_LANGUAGE:CUDA>:
        -Xcompiler=-Wall,-Wextra
        --expt-relaxed-constexpr  # Useful for constexpr in device code
    >
)

# ═════════════════════════════════════
#  Compile Definitions (Asset Paths)
# ═════════════════════════════════════
target_compile_definitions(${PROJECT_NAME} PRIVATE
    ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets"
    TEXTURES_DIR="${CMAKE_SOURCE_DIR}/assets/textures"
    MODELS_DIR="${CMAKE_SOURCE_DIR}/assets/models"
    SCENES_DIR="${CMAKE_SOURCE_DIR}/assets/scenes"
)

# ═════════════════════════════════════
#  Build Type Optimizations
# ═════════════════════════════════════
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Release optimizations
target_compile_options(${PROJECT_NAME} PRIVATE
    $<$<AND:$<CONFIG:Release>,$<COMPILE_LANGUAGE:CXX>>:
        $<$<CXX_COMPILER_ID:GNU,Clang>:-O3 -march=native -ffast-math>
    >
    $<$<AND:$<CONFIG:Release>,$<COMPILE_LANGUAGE:CUDA>>:
        -O3 --use_fast_math
    >
)

# Debug flags
target_compile_options(${PROJECT_NAME} PRIVATE
    $<$<CONFIG:Debug>:
        $<$<COMPILE_LANGUAGE:CXX>:-g -O0>
        $<$<COMPILE_LANGUAGE:CUDA>:-G -g>
    >
)

# ═════════════════════════════════════
#  Resource Macros
# ═════════════════════════════════════
target_compile_definitions(${PROJECT_NAME} PUBLIC
    IMAGE_DIR="${CMAKE_SOURCE_DIR}/assets/textures"
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
    MODEL_DIR="${CMAKE_SOURCE_DIR}/assets/models"
)

# ═════════════════════════════════════
#  Status Report
# ═════════════════════════════════════
message(STATUS "")
message(STATUS "══════════════════════════════════════")
message(STATUS "  Ray Tracer Configuration")
message(STATUS "══════════════════════════════════════")
message(STATUS "Build type:       ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard:     ${CMAKE_CXX_STANDARD}")
message(STATUS "CUDA enabled:     ${ENABLE_CUDA}")
if(ENABLE_CUDA)
    message(STATUS "CUDA standard:    ${CMAKE_CUDA_STANDARD}")
    message(STATUS "CUDA arch:        ${CMAKE_CUDA_ARCHITECTURES}")
endif()
message(STATUS "OpenMP found:     ${OpenMP_CXX_FOUND}")
message(STATUS "Build tests:      ${BUILD_TESTS}")
message(STATUS "Output dir:       ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "══════════════════════════════════════")
message(STATUS "")
