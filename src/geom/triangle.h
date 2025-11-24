#pragma once

#include "core/interval.h"
#include "core/vec3.h"
#include "core/ray.h"
#include "core/math_utils.h"

#include "hittable.h"

#include <memory>

namespace rt::geom {

class Triangle : public Hittable {
public:
    int gpu_index;

    Triangle(const core::Point3& a,
             const core::Point3& b,
             const core::Point3& c,
             std::shared_ptr<material::Material> mat);

    bool Hit(const core::Ray& r, core::Interval ray_t, HitRecord& rec) const override;
    Aabb BoundingBox() const override;

    int TypeId() const override;
    int ObjectIndex() const override;
    void SetObjIndex(int i) override;

private:
    core::Point3 a_, b_, c_;
    std::shared_ptr<material::Material> mat_;
    Aabb bbox_;
};

} // namespace rt::geom
