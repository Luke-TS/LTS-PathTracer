#include "texture.h"

#include <cmath>
#include <algorithm>

namespace rt::material {

// ======================= SolidColor =======================

SolidColor::SolidColor(const core::Color& albedo)
    : albedo_(albedo) {}

SolidColor::SolidColor(double red, double green, double blue)
    : albedo_(core::Color(red, green, blue)) {}

core::Color SolidColor::Value(double u, double v, const core::Point3& p) const {
    return albedo_;
}

// ======================= CheckerTexture =======================

CheckerTexture::CheckerTexture(double scale,
                               std::shared_ptr<Texture> even,
                               std::shared_ptr<Texture> odd)
    : inv_scale_(1.0 / scale), even_(even), odd_(odd) {}

CheckerTexture::CheckerTexture(double scale, const core::Color& c1, const core::Color& c2)
    : CheckerTexture(scale,
                     std::make_shared<SolidColor>(c1),
                     std::make_shared<SolidColor>(c2)) {}

core::Color CheckerTexture::Value(double u, double v, const core::Point3& p) const {
    int xInteger = int(std::floor(inv_scale_ * p.x()));
    int yInteger = int(std::floor(inv_scale_ * p.y()));
    int zInteger = int(std::floor(inv_scale_ * p.z()));

    bool isEven = ((xInteger + yInteger + zInteger) % 2 == 0);
    return isEven ? even_->Value(u, v, p) : odd_->Value(u, v, p);
}

// ======================= ImageTexture =======================

ImageTexture::ImageTexture(const char* filename)
    : image_(filename) {}

core::Color ImageTexture::Value(double u, double v, const core::Point3& p) const {
    if (image_.Height() <= 0) {
        return core::Color(0, 1, 1); // bright cyan debug color
    }

    // Clamp to [0,1], flip v
    u = core::Interval(0, 1).Clamp(u);
    v = 1.0 - core::Interval(0, 1).Clamp(v);

    int i = int(u * image_.Width());
    int j = int(v * image_.Height());

    auto pixel = image_.PixelData(i, j);

    const double scale = 1.0 / 255.0;
    return core::Color(scale * pixel[0], scale * pixel[1], scale * pixel[2]);
}

} // namespace rt::material
