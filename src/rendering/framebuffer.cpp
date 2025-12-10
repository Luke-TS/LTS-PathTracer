#include <cassert>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wmissing-braces"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "third-party/stb/stb_image_write.h"

#pragma GCC diagnostic pop

#include <fstream>

#include "framebuffer.h"

namespace rt::renderer {

Framebuffer::Framebuffer(int width, int height)
    : width_(width), height_(height)
{
    pixels_.resize(width * height, core::Color(0,0,0));
}

void Framebuffer::SetPixel(int x, int y, const core::Color& L)
{
    pixels_[y * width_ + x] = L;
}

void Framebuffer::SetPixel(int idx, const core::Color& L)
{
    assert(idx < width_ * height_);
    pixels_[idx] = L;
}

void Framebuffer::SavePNG(const std::string& filename) const
{
    std::vector<unsigned char> bytes(width_ * height_ * 3);

    for (int y = 0; y < height_; y++) {
        for (int x = 0; x < width_; x++) {

            const core::Color& c = pixels_[y * width_ + x];
            int idx = 3 * (y * width_ + x);

            bytes[idx + 0] = core::FloatToByte(c.x());
            bytes[idx + 1] = core::FloatToByte(c.y());
            bytes[idx + 2] = core::FloatToByte(c.z());
        }
    }

    stbi_write_png(
        filename.c_str(),
        width_,
        height_,
        3,
        bytes.data(),
        width_ * 3
    );

    std::clog << "Saved PNG: " << filename << "\n";
}

void Framebuffer::SavePPM(const std::string& filename) const
{
    std::ofstream out(filename);
    out << "P3\n" << width_ << " " << height_ << "\n255\n";

    for (int y = 0; y < height_; y++) {
        for (int x = 0; x < width_; x++) {

            const core::Color& c = pixels_[y * width_ + x];

            int r = int(core::FloatToByte(c.x()));
            int g = int(core::FloatToByte(c.y()));
            int b = int(core::FloatToByte(c.z()));

            out << r << " " << g << " " << b << "\n";
        }
    }

    std::clog << "Saved PPM: " << filename << "\n";
}

std::string ToLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](char c){
        return std::tolower(c);
    });
    return s;
}

void Framebuffer::Save(const std::string& filename) const {
    auto lower = ToLower(filename);

    if (lower.ends_with(".png"))
        return SavePNG(filename);
    if (lower.ends_with(".ppm"))
        return SavePPM(filename);
    else {
        std::clog << "Unknown output format - Saving as PPM."
            << "\nSupported: .png .ppm\n";
        return SavePPM(filename);
    }
}

} // namespace = rt::renderer

