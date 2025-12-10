#pragma once
#include <vector>
#include <string>
#include "core/color.h"

namespace rt::renderer {

class Framebuffer {
public:
    Framebuffer(int width, int height);

    int Width() const  { return width_; }
    int Height() const { return height_; }

    // Add a radiance sample to pixel (x,y)
    void SetPixel(int x, int y, const core::Color& L);
    void SetPixel(int idx, const core::Color& L);

    // Output
    void Save(const std::string& filename) const;

private:
    int width_, height_;
    std::vector<core::Color> pixels_;   // final averaged result

    void SavePNG(const std::string& filename) const;
    void SavePPM(const std::string& filename) const;
};

} // namespace rt::core
