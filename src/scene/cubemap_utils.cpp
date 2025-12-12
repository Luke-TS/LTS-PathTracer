// cubemap_utils.cpp
#include "cubemap_utils.h"
#include <stdexcept>
#include <algorithm>

namespace rt::scene {

// SubTexture wraps a portion of another texture
class SubTexture : public material::Texture {
private:
    std::shared_ptr<material::ImageTexture> source;
    int x_offset, y_offset;
    int face_width, face_height;
    int source_width, source_height;
    
public:
    SubTexture(std::shared_ptr<material::ImageTexture> src,
               int x_off, int y_off, int w, int h)
        : source(src), x_offset(x_off), y_offset(y_off),
          face_width(w), face_height(h)
    {
        source_width = src->Width();
        source_height = src->Height();
    }
    
    core::Color Value(double u, double v, const core::Point3& p) const override {
        // Clamp u and v to [0, 1]
        u = std::max(0.0, std::min(1.0, u));
        v = std::max(0.0, std::min(1.0, v));
        
        // Map UV from face [0,1] to source texture coordinates
        double source_u = (x_offset + u * face_width) / static_cast<double>(source_width);
        double source_v = (y_offset + v * face_height) / static_cast<double>(source_height);
        
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
LoadCubemapFromFiles(const std::array<std::string, 6>& paths)
{
    std::array<std::shared_ptr<material::Texture>, 6> faces;
    
    for (int i = 0; i < 6; ++i) {
        faces[i] = std::make_shared<material::ImageTexture>(paths[i]);
    }
    
    return faces;
}

std::array<std::shared_ptr<material::Texture>, 6>
ExtractCubemapFaces(const std::string& path, CubemapLayout layout)
{
    if (layout == CubemapLayout::SEPARATE_FILES) {
        throw std::runtime_error(
            "SEPARATE_FILES layout requires using LoadCubemapFromFiles() instead");
    }
    
    auto full_image = std::make_shared<material::ImageTexture>(path);
    int img_w = full_image->Width();
    int img_h = full_image->Height();
    
    std::array<std::shared_ptr<material::Texture>, 6> faces;
    
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
