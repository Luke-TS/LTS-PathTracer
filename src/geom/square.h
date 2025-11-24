#pragma once

#include "core/math_utils.h"
#include "core/interval.h"
#include "core/vec3.h"
#include "hittable.h"

#include <memory>

namespace rt::geom {

class Square : public Hittable {
public:
    int gpu_index;

    Square(const core::Point3& a, const core::Point3& b,
           const core::Point3& c, const core::Point3& d,
           std::shared_ptr<material::Material> mat);

    bool Hit(const core::Ray& r, core::Interval ray_t, HitRecord& rec) const override;
    Aabb BoundingBox() const override;

    int TypeId() const override;
    int ObjectIndex() const override;
    void SetObjIndex(int i) override;

private:
    core::Point3 a_, b_, c_, d_;
    std::shared_ptr<material::Material> mat_;
    Aabb bbox_;
};

} // namespace rt::geom
