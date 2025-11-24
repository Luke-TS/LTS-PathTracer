#include "square.h"

namespace rt::geom {

Square::Square(const core::Point3& a, const core::Point3& b,
               const core::Point3& c, const core::Point3& d,
               std::shared_ptr<material::Material> mat)
    : a_(a), b_(b), c_(c), d_(d), mat_(mat) {

    core::Point3 min_point(
        std::fmin(a.x(), b.x()),
        std::fmin(a.y(), b.y()),
        std::fmin(a.z(), b.z())
    );

    core::Point3 max_point(
        std::fmax(a.x(), b.x()),
        std::fmax(a.y(), b.y()),
        std::fmax(a.z(), b.z())
    );

    const double eps = 1e-6;
    min_point += -core::Vec3(eps, eps, eps);
    max_point += core::Vec3(eps, eps, eps);

    bbox_ = Aabb(min_point, max_point);
}

bool Square::Hit(const core::Ray& r, core::Interval ray_t, HitRecord& rec) const {
    const float kEps = 1e-6f;

    core::Vec3 edge1 = b_ - a_;
    core::Vec3 edge2 = c_ - a_;

    core::Vec3 pvec = Cross(r.direction(), edge2);
    float det = Dot(edge1, pvec);
    if (fabs(det) < kEps)
        return false;

    float inv_det = 1.0f / det;
    core::Vec3 tvec = r.origin() - a_;

    float u = Dot(tvec, pvec) * inv_det;
    if (u < 0.0f || u > 1.0f)
        return false;

    core::Vec3 qvec = Cross(tvec, edge1);
    float v = Dot(r.direction(), qvec) * inv_det;
    if (v < 0.0f || (u + v) > 1.0f)
        return false;

    float t = Dot(edge2, qvec) * inv_det;
    if (t < ray_t.min_ || t > ray_t.max_)
        return false;

    rec.t = t;
    rec.p = r.at(t);
    rec.mat = mat_;

    core::Vec3 outward_norm = Cross(edge1, edge2);
    rec.SetFaceNorm(r, core::Normalize(outward_norm));

    return true;
}

Aabb Square::BoundingBox() const {
    return bbox_;
}

int Square::TypeId() const {
    return HITTABLE_SQUARE;
}

int Square::ObjectIndex() const {
    return gpu_index;
}

void Square::SetObjIndex(int i) {
    gpu_index = i;
}

} // namespace rt::geom
