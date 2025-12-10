#include "rect.h"

namespace rt::geom {

// -------------------
// xy_rect
// -------------------

xy_rect::xy_rect() {}

xy_rect::xy_rect(double x0, double x1, double y0, double y1, double k,
                 std::shared_ptr<material::Material> mat)
    : mat_(mat), x0_(x0), x1_(x1), y0_(y0), y1_(y1), k_(k) {}

bool xy_rect::Hit(const core::Ray& r, core::Interval ray_t, HitRecord& rec) const {
    auto t = (k_ - r.origin().z()) / r.direction().z();
    if (!ray_t.Surrounds(t)) return false;

    auto x = r.origin().x() + t * r.direction().x();
    auto y = r.origin().y() + t * r.direction().y();

    if (x < x0_ || x > x1_ || y < y0_ || y > y1_) return false;

    rec.u = (x - x0_) / (x1_ - x0_);
    rec.v = (y - y0_) / (y1_ - y0_);
    rec.t = t;

    core::Vec3 outward_normal(0, 0, 1);
    rec.SetFaceNorm(r, outward_normal);
    rec.mat = mat_;
    rec.p = r.at(t);

    return true;
}

Aabb xy_rect::BoundingBox() const {
    return Aabb(
        core::Point3(x0_, y0_, k_ - 0.0001),
        core::Point3(x1_, y1_, k_ + 0.0001)
    );
}


// -------------------
// xz_rect
// -------------------

xz_rect::xz_rect() {}

xz_rect::xz_rect(double x0, double x1, double z0, double z1, double k,
                 std::shared_ptr<material::Material> mat)
    : mat_(mat), x0_(x0), x1_(x1), z0_(z0), z1_(z1), k_(k) {}

bool xz_rect::Hit(const core::Ray& r, core::Interval ray_t, HitRecord& rec) const {
    auto t = (k_ - r.origin().y()) / r.direction().y();
    if (!ray_t.Surrounds(t)) return false;

    auto x = r.origin().x() + t * r.direction().x();
    auto z = r.origin().z() + t * r.direction().z();

    if (x < x0_ || x > x1_ || z < z0_ || z > z1_) return false;

    rec.u = (x - x0_) / (x1_ - x0_);
    rec.v = (z - z0_) / (z1_ - z0_);
    rec.t = t;

    core::Vec3 outward_normal(0, 1, 0);
    rec.SetFaceNorm(r, outward_normal);
    rec.mat = mat_;
    rec.p = r.at(t);

    return true;
}

Aabb xz_rect::BoundingBox() const {
    return Aabb(
        core::Point3(x0_, k_ - 0.0001, z0_),
        core::Point3(x1_, k_ + 0.0001, z1_)
    );
}


// -------------------
// yz_rect
// -------------------

yz_rect::yz_rect() {}

yz_rect::yz_rect(double y0, double y1, double z0, double z1, double k,
                 std::shared_ptr<material::Material> mat)
    : mat_(mat), y0_(y0), y1_(y1), z0_(z0), z1_(z1), k_(k) {}

bool yz_rect::Hit(const core::Ray& r, core::Interval ray_t, HitRecord& rec) const {
    auto t = (k_ - r.origin().x()) / r.direction().x();
    if (!ray_t.Surrounds(t)) return false;

    auto y = r.origin().y() + t * r.direction().y();
    auto z = r.origin().z() + t * r.direction().z();

    if (y < y0_ || y > y1_ || z < z0_ || z > z1_) return false;

    rec.u = (y - y0_) / (y1_ - y0_);
    rec.v = (z - z0_) / (z1_ - z0_);
    rec.t = t;

    core::Vec3 outward_normal(1, 0, 0);
    rec.SetFaceNorm(r, outward_normal);
    rec.mat = mat_;
    rec.p = r.at(t);

    return true;
}

Aabb yz_rect::BoundingBox() const {
    return Aabb(
        core::Point3(k_ - 0.0001, y0_, z0_),
        core::Point3(k_ + 0.0001, y1_, z1_)
    );
}

} // namespace rt::geom
