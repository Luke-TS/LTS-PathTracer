#pragma once

#include "core/color.h"
#include "material/texture.h"
#include <memory>

namespace rt::scene {

class Environment {
public:
    virtual ~Environment() = default;
    virtual core::Color sample(const core::Vec3& direction) const = 0;
};

class GradientEnvironment : public Environment {
private:
    core::Color bottom_color;
    core::Color top_color;
    
public:
    GradientEnvironment(const core::Color& bottom, const core::Color& top)
        : bottom_color(bottom), top_color(top) {}
    
    core::Color sample(const core::Vec3& direction) const override {
        float t = 0.5f * (direction.y() + 1.0f);
        return (1.0f - t) * bottom_color + t * top_color;
    }
};

class CubemapEnvironment : public Environment {
private:
    std::shared_ptr<material::Texture> faces[6];  // +X, -X, +Y, -Y, +Z, -Z
    float intensity;
    
    enum Face { POS_X = 0, NEG_X = 1, POS_Y = 2, NEG_Y = 3, POS_Z = 4, NEG_Z = 5 };
    
    Face get_face(const core::Vec3& dir) const {
        core::Vec3 abs_dir(fabs(dir.x()), fabs(dir.y()), fabs(dir.z()));
        
        if (abs_dir.x() >= abs_dir.y() && abs_dir.x() >= abs_dir.z()) {
            return (dir.x() > 0) ? POS_X : NEG_X;
        }
        else if (abs_dir.y() >= abs_dir.x() && abs_dir.y() >= abs_dir.z()) {
            return (dir.y() > 0) ? POS_Y : NEG_Y;
        }
        else {
            return (dir.z() > 0) ? POS_Z : NEG_Z;
        }
    }
    
    std::pair<float, float> dir_to_uv(const core::Vec3& dir, Face face) const {
        float u, v;
        
        switch(face) {
            case POS_X:  // Right (+X)
                u = -dir.z() / dir.x();
                v = -dir.y() / dir.x();
                break;
            case NEG_X:  // Left (-X)
                u = dir.z() / -dir.x();
                v = -dir.y() / -dir.x();
                break;
            case POS_Y:  // Top (+Y)
                u = dir.x() / dir.y();
                v = dir.z() / dir.y();
                break;
            case NEG_Y:  // Bottom (-Y)
                u = dir.x() / -dir.y();
                v = -dir.z() / -dir.y();
                break;
            case POS_Z:  // Front (+Z)
                u = dir.x() / dir.z();
                v = -dir.y() / dir.z();
                break;
            case NEG_Z:  // Back (-Z)
                u = -dir.x() / -dir.z();
                v = -dir.y() / -dir.z();
                break;
        }
        
        // Convert from [-1, 1] to [0, 1]
        u = (u + 1.0f) * 0.5f;
        v = (v + 1.0f) * 0.5f;
        
        return {u, v};
    }
    
public:
    CubemapEnvironment(std::shared_ptr<material::Texture> cube_faces[6], float intens = 1.0f)
        : intensity(intens) {
        for (int i = 0; i < 6; ++i) {
            faces[i] = cube_faces[i];
        }
    }
    
    core::Color sample(const core::Vec3& direction) const override {
        Face face = get_face(direction);
        auto [u, v] = dir_to_uv(direction, face);
        
        return faces[face]->Value(u, v, core::Point3()) * intensity;
    }
};

class SolidEnvironment : public Environment {
private:
    core::Color color;
    
public:
    SolidEnvironment(const core::Color& c) : color(c) {}
    
    core::Color sample(const core::Vec3& direction) const override {
        return color;
    }
};

} // namespace rt::scene
