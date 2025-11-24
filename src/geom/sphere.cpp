#include "sphere.h"

namespace rt::geom {

Sphere::Sphere(const core::Point3& center, double radius,
               std::shared_ptr<material::Material> mat)
    : center_(center), radius_(std::fmax(0, radius)), mat_(mat) 
{
    core::Vec3 rv(radius, radius, radius);
    bbox_ = Aabb(core::Point3(center + rv), core::Point3(center - rv));
}

bool Sphere::Hit(const core::Ray& r, core::Interval ray_t, HitRecord& rec) const {
    core::Vec3 oc = center_ - r.origin();
    auto a = r.direction().length_squared();
    auto h = Dot(r.direction(), oc);
    auto c = oc.length_squared() - radius_ * radius_;

    auto discriminant = h*h - a*c;
    if (discriminant < 0) return false;

    auto sqrtd = std::sqrt(discriminant);

    auto root = (h - sqrtd) / a;
    if (!ray_t.Surrounds(root)) {
        root = (h + sqrtd) / a;
        if (!ray_t.Surrounds(root)) return false;
    }

    rec.t = root;
    rec.p = r.at(rec.t);

    core::Vec3 outward_normal = (rec.p - center_) / radius_;
    rec.SetFaceNorm(r, outward_normal);
    get_sphere_uv(outward_normal, rec.u, rec.v);
    rec.mat = mat_;

    return true;
}

Aabb Sphere::BoundingBox() const { return bbox_; }

int Sphere::TypeId() const { return HITTABLE_SPHERE; }

int Sphere::ObjectIndex() const { return gpu_index; }

void Sphere::SetObjIndex(int i) { gpu_index = i; }

void Sphere::get_sphere_uv(const core::Point3& p, double& u, double& v) {
    auto theta = std::acos(-p.y());
    auto phi = std::atan2(-p.z(), p.x()) + core::kPi;

    u = phi / (2 * core::kPi);
    v = theta / core::kPi;
}

} // namespace rt::geom
