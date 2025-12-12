// cubemap_utils.h
#pragma once
#include "textures/texture.h"
#include <memory>
#include <string>
#include <array>

namespace rt::scene {

enum class CubemapLayout {
    HORIZONTAL_CROSS,  // 4x3 grid
    VERTICAL_CROSS,    // 3x4 grid
    HORIZONTAL_STRIP,  // 6x1
    VERTICAL_STRIP,    // 1x6
    SEPARATE_FILES     // Individual face files
};

// Extract 6 face textures from a single cubemap image
std::array<std::shared_ptr<material::Texture>, 6>
ExtractCubemapFaces(const std::string& path, CubemapLayout layout);

// Load cubemap from 6 separate files
// Expects files in order: +X, -X, +Y, -Y, +Z, -Z (right, left, top, bottom, front, back)
std::array<std::shared_ptr<material::Texture>, 6>
LoadCubemapFromFiles(const std::array<std::string, 6>& paths);

// Helper to create a subtexture from a region of an image
std::shared_ptr<material::Texture>
CreateSubTexture(std::shared_ptr<material::ImageTexture> source,
                 int x_offset, int y_offset,
                 int width, int height);

} // namespace rt::scene
