// cubemap_utils.h
#pragma once
#include "material/texture.h"
#include <memory>
#include <string>

namespace rt::scene {

enum class CubemapLayout {
    HORIZONTAL_CROSS,  // 4x3 grid
    VERTICAL_CROSS,    // 3x4 grid
    HORIZONTAL_STRIP,  // 6x1 or 1x6
    VERTICAL_STRIP,    // 1x6 or 6x1
    SEPARATE_FILES     // Individual face files
};

// Extract 6 face textures from a single cubemap image
std::array<std::shared_ptr<material::Texture>, 6>
ExtractCubemapFaces(const std::string& path, CubemapLayout layout);

// Helper to create a subtexture from a region of an image
std::shared_ptr<material::Texture>
CreateSubTexture(std::shared_ptr<material::ImageTexture> source,
                 int x_offset, int y_offset,
                 int width, int height);

} // namespace rt::scene


// cubemap_utils.cpp
#include "cubemap_utils.h"
#include <stdexcept>

namespace rt::scene {

// SubTexture wraps a portion of another texture
class SubTexture : public material::Texture {
private:
    std::shared_ptr<material::Texture> source;
    int x_offset, y_offset;
    int face_width, face_height;
    int source_width, source_height;
    
public:
    SubTexture(std::shared_ptr<material::Texture> src,
               int x_off, int y_off, int w, int h)
        : source(src), x_offset(x_off), y_offset(y_off),
          face_width(w), face_height(h)
    {
        // Get source dimensions from the texture
        // You'll need to add width/height getters to ImageTexture
        source_width = src->Width();
        source_height = src->Height();
    }
    
    core::Color Value(double u, double v, const core::Point3& p) const override {
        // Map UV from face [0,1] to source texture coordinates
        double source_u = (x_offset + u * face_width) / source_width;
        double source_v = (y_offset + v * face_height) / source_height;
        
        return source->Value(source_u, source_v, p);
    }
};

std::shared_ptr<material::Texture>
CreateSubTexture(std::shared_ptr<material::ImageTexture> source,
                 int x_offset, int y_offset,
                 int width, int height)
{
    return std::make_shared<SubTexture>(source, x_offset, y_offset, width, height);
}

std::array<std::shared_ptr<material::Texture>, 6>
ExtractCubemapFaces(const std::string& path, CubemapLayout layout)
{
    auto full_image = std::make_shared<material::ImageTexture>(path);
    int img_w = full_image->width();
    int img_h = full_image->height();
    
    std::array<std::shared_ptr<material::ImageTexture>, 6> faces;
    
    switch (layout) {
        case CubemapLayout::HORIZONTAL_CROSS: {
            // Layout:
            //       [top]
            // [left][front][right][back]
            //       [bottom]
            // 4x3 grid, each face is img_w/4 x img_h/3
            
            int face_size = img_w / 4;
            
            if (img_w != face_size * 4 || img_h != face_size * 3) {
                throw std::runtime_error("Invalid horizontal cross dimensions");
            }
            
            faces[0] = CreateSubTexture(full_image, face_size * 2, face_size, face_size, face_size);     // +X (right)
            faces[1] = CreateSubTexture(full_image, 0, face_size, face_size, face_size);                 // -X (left)
            faces[2] = CreateSubTexture(full_image, face_size, 0, face_size, face_size);                 // +Y (top)
            faces[3] = CreateSubTexture(full_image, face_size, face_size * 2, face_size, face_size);     // -Y (bottom)
            faces[4] = CreateSubTexture(full_image, face_size, face_size, face_size, face_size);         // +Z (front)
            faces[5] = CreateSubTexture(full_image, face_size * 3, face_size, face_size, face_size);     // -Z (back)
            break;
        }
        
        case CubemapLayout::VERTICAL_CROSS: {
            // Layout:
            //       [back]
            // [left][top][right]
            //       [front]
            //       [bottom]
            // 3x4 grid
            
            int face_size = img_w / 3;
            
            if (img_w != face_size * 3 || img_h != face_size * 4) {
                throw std::runtime_error("Invalid vertical cross dimensions");
            }
            
            faces[0] = CreateSubTexture(full_image, face_size * 2, face_size, face_size, face_size);     // +X (right)
            faces[1] = CreateSubTexture(full_image, 0, face_size, face_size, face_size);                 // -X (left)
            faces[2] = CreateSubTexture(full_image, face_size, face_size, face_size, face_size);         // +Y (top)
            faces[3] = CreateSubTexture(full_image, face_size, face_size * 3, face_size, face_size);     // -Y (bottom)
            faces[4] = CreateSubTexture(full_image, face_size, face_size * 2, face_size, face_size);     // +Z (front)
            faces[5] = CreateSubTexture(full_image, face_size, 0, face_size, face_size);                 // -Z (back)
            break;
        }
        
        case CubemapLayout::HORIZONTAL_STRIP: {
            // Layout: [right][left][top][bottom][front][back] in a row
            // 6x1 grid
            
            int face_size = img_w / 6;
            
            if (img_h != face_size) {
                throw std::runtime_error("Invalid horizontal strip dimensions");
            }
            
            for (int i = 0; i < 6; ++i) {
                faces[i] = CreateSubTexture(full_image, i * face_size, 0, face_size, face_size);
            }
            break;
        }
        
        case CubemapLayout::VERTICAL_STRIP: {
            // Layout: stacked vertically
            // 1x6 grid
            
            int face_size = img_h / 6;
            
            if (img_w != face_size) {
                throw std::runtime_error("Invalid vertical strip dimensions");
            }
            
            for (int i = 0; i < 6; ++i) {
                faces[i] = CreateSubTexture(full_image, 0, i * face_size, face_size, face_size);
            }
            break;
        }
        
        default:
            throw std::runtime_error("Unsupported cubemap layout");
    }
    
    return faces;
}

} // namespace rt::scene
