cmake_minimum_required(VERSION 3.18)

project(ray_tracer LANGUAGES CXX CUDA)


# ───────────────── Compiler Settings ──
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
set(CMAKE_CUDA_ARCHITECTURES native CACHE STRING "GPU arch")

# Always export compile_commands.json for clangd lsp
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ───────────────── Source Collection ─
file(GLOB_RECURSE CPP_SOURCES
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/*.cc"
)

file(GLOB_RECURSE CUDA_SOURCES
    "${CMAKE_SOURCE_DIR}/src/*.cu"
)

add_executable(${PROJECT_NAME}
    ${CPP_SOURCES}
    ${CUDA_SOURCES}
)

# ───────────────── Includes ─────
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
    SYSTEM PRIVATE
        ${CMAKE_SOURCE_DIR}/external/tinyobjloader
)

# ───────────────── Dependencies ────
set(TINYOBJLOADER_BUILD_TEST OFF)
add_subdirectory(external/tinyobjloader EXCLUDE_FROM_ALL)

# JSON lib
include(FetchContent)
FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(json)

find_package(OpenMP REQUIRED)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        OpenMP::OpenMP_CXX
        tinyobjloader
        nlohmann_json::nlohmann_json
)

# ───────────────── Warning Levels ─
target_compile_options(${PROJECT_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wpedantic>
    $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-Wall -Xcompiler=-Wextra>
)

# ─────── Asset Directory Macro ──────────
target_compile_definitions(${PROJECT_NAME} PUBLIC
    IMAGE_DIR="${CMAKE_SOURCE_DIR}/assets/textures"
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
    MODEL_DIR="${CMAKE_SOURCE_DIR}/assets/models"
)
