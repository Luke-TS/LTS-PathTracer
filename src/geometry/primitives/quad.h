#pragma once
#include "geom/hittable.h"
#include "geom/triangle.h"

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
    std::shared_ptr<Triangle> t1, t2;
    int obj_idx_ = -1;
};

} // namespace rt::geom

