#pragma once

#include "interval.h"
#include "vec3.h"
#include <algorithm>

namespace rt::core {

using Color = Vec3;

inline float LinearToGamma(double linear_component) {
    if( linear_component > 0 ) {
        return std::sqrt(linear_component);
    }

    return 0;
}

inline unsigned char FloatToByte(float x) {
    x = std::clamp(LinearToGamma(x), 0.0f, 0.999f);
    return static_cast<unsigned char>(256.f * x);
}

inline double Luminance(const Color& c) {
    return 0.2126f * c.x() + 0.7152f * c.y() + 0.0722f * c.z();
}

} // namespace rt::core
