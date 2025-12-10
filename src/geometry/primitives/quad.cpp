#include "geom/quad.h"
#include "geom/triangle.h"

namespace rt::geom {

Quad::Quad(const core::Point3& a,
           const core::Point3& b,
           const core::Point3& c,
           const core::Point3& d,
           std::shared_ptr<material::Material> mat)
{
    t1 = std::make_shared<Triangle>(a, b, c, mat);
    t2 = std::make_shared<Triangle>(a, c, d, mat);
}

bool Quad::Hit(const core::Ray& r, core::Interval t, HitRecord& rec) const {
    HitRecord tmp;
    if (t1->Hit(r, t, tmp)) { rec = tmp; return true; }
    if (t2->Hit(r, t, tmp)) { rec = tmp; return true; }
    return false;
}

Aabb Quad::BoundingBox() const {
    return Aabb(t1->BoundingBox(), t2->BoundingBox());
}

} // namespace rt::geom

