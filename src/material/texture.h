#pragma once

#include "core/color.h"
#include "core/interval.h"
#include "scene/image.h"

#include <memory>

namespace rt::material {

class Texture {
public:
    virtual ~Texture() = default;
    virtual core::Color Value(double u, double v, const core::Point3& p) const = 0;
};

// -------------------- SolidColor --------------------

class SolidColor : public Texture {
public:
    SolidColor(const core::Color& albedo);
    SolidColor(double red, double green, double blue);

    core::Color Value(double u, double v, const core::Point3& p) const override;

private:
    core::Color albedo_;
};

// -------------------- CheckerTexture --------------------

class CheckerTexture : public Texture {
public:
    CheckerTexture(double scale, std::shared_ptr<Texture> even, std::shared_ptr<Texture> odd);
    CheckerTexture(double scale, const core::Color& c1, const core::Color& c2);

    core::Color Value(double u, double v, const core::Point3& p) const override;

private:
    double inv_scale_;
    std::shared_ptr<Texture> even_;
    std::shared_ptr<Texture> odd_;
};

// -------------------- ImageTexture --------------------

class ImageTexture : public Texture {
public:
    explicit ImageTexture(std::string filename);

    core::Color Value(double u, double v, const core::Point3& p) const override;

private:
    scene::Image image_;
};

} // namespace rt::material
