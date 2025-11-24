#pragma once

#include <cmath>
#include <omp.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <sched.h>
#include <unordered_map>

#include "core/color.h"
#include "core/constants.h"
#include "core/math_utils.h"
#include "core/interval.h"
#include "core/vec3.h"

#include "geom/hittable.h"
#include "material/material.h"

namespace rt::scene {

using json = nlohmann::json;

struct CameraConfig {
    double aspect_ratio = 16/9.0;
    int    image_width  = 400;
    int    samples_per_pixel = 50;
    int    max_depth    = 10;

    double vfov = 90.0;
    core::Vec3   lookfrom = core::Point3(0,0,0);
    core::Vec3   lookat   = core::Point3(0,0,-1);
    core::Vec3   vup      = core::Point3(0,1,0);

    double defocus_angle = 0.0;
    double focus_dist    = 10.0;
};

CameraConfig parseCamera(const json& j);
std::unordered_map<std::string, CameraConfig> loadCameras(const std::string& filename);

class Camera {
public:
    double aspect_ratio_ = 1.0;
    int    image_width_  = 100;
    int    max_depth_ = 10;
    int    samples_per_pixel_ = 10;

    double vfov_ = 90.0;
    core::Vec3   lookfrom_ = core::Point3(0,0,0);
    core::Vec3   lookat    = core::Point3(0,0,-1);
    core::Vec3   vup_      = core::Point3(0,1,0);

    double defocus_angle_ = 0;
    double focus_dist_ = 10;

    void SetFromConfig(const CameraConfig& cfg);
    void Initialize();

    virtual core::Ray GetRay(int i, int j) const;

    virtual core::Color GetPixel(const core::Ray& r, int depth, const geom::Hittable& world) const;

    int get_image_height() const { return image_height_; }
    int get_image_width() const { return image_width_; }

protected:
    int    image_height_;
    core::Point3 center_;
    core::Point3 pixel00_loc_;
    core::Vec3 pixel_delta_u_;
    core::Vec3 pixel_delta_v_;
    core::Vec3 u_, v_, w_;
    core::Vec3 defocus_disk_u_;
    core::Vec3 defocus_disk_v_;

    core::Point3 defocus_disk_sample() const;
    core::Vec3 SampleSquare() const;
};

class ColorCamera : public Camera {};

class DepthCamera : public Camera {
public:
    virtual core::Color GetPixel(const core::Ray& r, int depth, const geom::Hittable& world) const override;

private:
    int max_dist_ = 20;
};

} // namespace rt::scene
