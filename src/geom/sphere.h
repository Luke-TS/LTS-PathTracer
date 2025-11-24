#pragma once

#include "constants.h"
#include "core/interval.h"
#include "core/vec3.h"
#include "hittable.h"
#include "aabb.h"

#include <memory>

namespace rt::geom {

class Sphere : public Hittable {
public:
    int gpu_index = -1;

    Sphere(const core::Point3& center, double radius,
           std::shared_ptr<material::Material> mat);

    bool Hit(const core::Ray& r, core::Interval ray_t, HitRecord& rec) const override;
    Aabb BoundingBox() const override;

    int TypeId() const override;
    int ObjectIndex() const override;
    void SetObjIndex(int i) override;

    static void get_sphere_uv(const core::Point3& p, double& u, double& v);

private:
    core::Point3 center_;
    double radius_;
    std::shared_ptr<material::Material> mat_;
    Aabb bbox_;
};

} // namespace rt::geom
