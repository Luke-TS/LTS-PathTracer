#include "triangle.h"

#include "core/constants.h"

namespace rt::geom {

Triangle::Triangle(const core::Point3& a,
                   const core::Point3& b,
                   const core::Point3& c,
                   std::shared_ptr<material::Material> mat)
    : a_(a), b_(b), c_(c), mat_(mat)
{
    core::Point3 min_point(
        std::fmin(a.x(), std::fmin(b.x(), c.x())),
        std::fmin(a.y(), std::fmin(b.y(), c.y())),
        std::fmin(a.z(), std::fmin(b.z(), c.z()))
    );

    core::Point3 max_point(
        std::fmax(a.x(), std::fmax(b.x(), c.x())),
        std::fmax(a.y(), std::fmax(b.y(), c.y())),
        std::fmax(a.z(), std::fmax(b.z(), c.z()))
    );

    const double eps = 1e-6;
    min_point += -core::Vec3(eps, eps, eps);
    max_point +=  core::Vec3(eps, eps, eps);

    bbox_ = Aabb(min_point, max_point);
}

bool Triangle::Hit(const core::Ray& r, core::Interval ray_t, HitRecord& rec) const {
    core::Vec3 edge1 = b_ - a_;
    core::Vec3 edge2 = c_ - a_;

    core::Vec3 pvec = core::Cross(r.direction(), edge2);
    float det = core::Dot(edge1, pvec);

    if (fabs(det) < core::kEpsilon)
        return false;

    float inv_det = 1.0f / det;
    core::Vec3 tvec = r.origin() - a_;

    float u = core::Dot(tvec, pvec) * inv_det;
    if (u < 0.0f || u > 1.0f)
        return false;

    core::Vec3 qvec = core::Cross(tvec, edge1);
    float v = core::Dot(r.direction(), qvec) * inv_det;
    if (v < 0.0f || (u + v) > 1.0f)
        return false;

    float t = core::Dot(edge2, qvec) * inv_det;

    if (t < ray_t.min_ || t > ray_t.max_)
        return false;

    rec.t = t;
    rec.p = r.at(t);
    rec.mat = mat_;

    core::Vec3 outward_norm = core::Cross(edge1, edge2);
    rec.SetFaceNorm(r, core::Normalize(outward_norm));

    return true;
}

Aabb Triangle::BoundingBox() const {
    return bbox_;
}

int Triangle::TypeId() const {
    return HITTABLE_TRIANGLE;
}

int Triangle::ObjectIndex() const {
    return gpu_index;
}

void Triangle::SetObjIndex(int i) {
    gpu_index = i;
}

} // namespace rt::geom
