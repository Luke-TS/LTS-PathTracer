#pragma once
#include <string>
#include <vector>

namespace rt::material {

class TextureImage {
 public:
  TextureImage() = default;
  explicit TextureImage(const std::string& filename);
  
  // New: create blank image for rendering
  TextureImage(int w, int h, bool allocateFloatPixels);

  bool Load(const std::string& filename);

  void SavePNG(const std::string& filename) const;   // NEW

  int Width() const { return width_; }
  int Height() const { return height_; }

  const unsigned char* PixelData(int x, int y) const;

  // New: get/set float pixel reference
  float* FloatPixel(int x, int y) { return &fdata_[(y * width_ + x) * 3]; }
  const float* FloatPixel(int x, int y) const { return &fdata_[(y * width_ + x) * 3]; }

 private:
  static int Clamp(int x, int low, int high);
  static unsigned char FloatToByte(float value);
  void ConvertToBytes();

  int width_ = 0;
  int height_ = 0;

  std::vector<float> fdata_;           // RGB float (0–1), used for rendering + HDR textures
  std::vector<unsigned char> bdata_;   // 8-bit RGB, derived from fdata_
};

}  // namespace rt::scene

