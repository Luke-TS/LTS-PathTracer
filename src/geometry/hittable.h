#pragma once

#include <memory>

#include "core/ray.h"
#include "core/vec3.h"
#include "core/interval.h"

#include "aabb.h"

// to solve circular references between material and hittable code
namespace rt::material {
    class Material;
}

namespace rt::geom {

class HitRecord {
public:
    bool hit;
    core::Point3 p; // hit point
    core::Vec3 normal; // normal vector
    std::shared_ptr<material::Material> mat;
    double t; // time of hit
    bool front_face;

    // surface coordinates of hit point for texture mapping
    double u; 
    double v;

    void SetFaceNorm(const core::Ray& r, const core::Vec3& out_norm) {
        front_face = core::Dot(r.direction(), out_norm) < 0;
        normal = front_face ? out_norm : -out_norm;
    }
};

// used to identify object type for GPU intersection testing
enum HittableType {
    HITTABLE_SPHERE = 0,
    HITTABLE_TRIANGLE = 1,
    HITTABLE_SQUARE = 2,
};;

class Hittable {
public:
    virtual ~Hittable() = default;

    virtual bool Hit(const core::Ray& r, core::Interval ray_t, HitRecord& rec) const = 0;

    virtual Aabb BoundingBox() const = 0;

    virtual int TypeId() const = 0;
    virtual int ObjectIndex() const = 0;
    virtual void SetObjIndex(int i) = 0;
};

} // namespace rt::geom
