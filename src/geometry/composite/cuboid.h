#pragma once
#include <memory>
#include <vector>

#include "geometry/hittable.h"
#include "geometry/primitives/triangle.h"
#include "geometry/accel/aabb.h"
#include "core/math/vec3.h"
#include "core/types/ray.h"

namespace rt::geom {

class Cuboid : public Hittable {
public:
    Cuboid(const core::Point3& min_pt,
           const core::Point3& max_pt,
           std::shared_ptr<material::Material> mat);

    bool Hit(const core::Ray& r, core::Interval ray_t, HitRecord& rec) const override;
    Aabb BoundingBox() const override { return bbox_; }

    int TypeId() const override { return 1337; }  // Any ID you want
    int ObjectIndex() const override { return obj_idx_; }
    void SetObjIndex(int i) override { obj_idx_ = i; }

private:
    std::vector<std::shared_ptr<Triangle>> faces_;
    Aabb bbox_;
    int obj_idx_ = -1;
};

} // namespace rt::geom
