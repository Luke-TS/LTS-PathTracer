#pragma once
#include "geometry/hittable.h"
#include "core/math/vec3.h"

namespace rt::geom {

class Quad : public Hittable {
public:
    Quad(const core::Point3& a,
         const core::Point3& b,
         const core::Point3& c,
         const core::Point3& d,
         std::shared_ptr<material::Material> mat);

    bool Hit(const core::Ray& r, core::Interval t, HitRecord& rec) const override;
    Aabb BoundingBox() const override;
    int TypeId() const override { return 777; }
    int ObjectIndex() const override { return obj_idx_; }
    void SetObjIndex(int i) override { obj_idx_ = i; }

private:
    core::Point3 a_, b_, c_, d_;  // Four corners
    core::Vec3 normal_;            // Plane normal
    double D_;                     // Plane equation: dot(normal, p) = D
    core::Vec3 u_, v_;            // Basis vectors for the quad
    double area_;                  // Area for sampling
    std::shared_ptr<material::Material> mat_;
    Aabb bbox_;
    int obj_idx_ = -1;
};

} // namespace rt::geom
