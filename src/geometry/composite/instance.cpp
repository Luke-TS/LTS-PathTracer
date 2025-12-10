#include "geom/instance.h"

#include "core/math_utils.h"
#include "geom/hittable.h"
#include "core/mat4.h"
#include "core/ray.h"
#include "geom/aabb.h"

namespace rt::geom {

Instance::Instance(const std::shared_ptr<Hittable>& obj,
                   const core::Mat4& M_,
                   const core::Mat4& invM_)
    : object(obj),
      M(M_),
      invM(invM_),
      invM_T(invM_.Transpose())
{}


// -------------------------------------------------------------
// HIT
// -------------------------------------------------------------
bool Instance::Hit(const core::Ray& r_world,
                   core::Interval ray_t,
                   HitRecord& rec) const
{
    // Transform ray into object space
    core::Ray r_object(
        invM * r_world.origin(),
        invM.TransformDirection(r_world.direction())
    );

    if (!object->Hit(r_object, ray_t, rec))
        return false;

    // Transform intersection back to world space
    rec.p = M * rec.p;

    // Transform normal by inverse-transpose
    rec.normal = core::Normalize(
        invM_T.TransformDirection(rec.normal)
    );

    return true;
}


// -------------------------------------------------------------
// BOUNDING BOX
// -------------------------------------------------------------
Aabb Instance::BoundingBox() const {
    // Get object-space bounding box
    Aabb box = object->BoundingBox();

    // Build 8 corners
    core::Point3 c[8] = {
        { box.x.min_, box.y.min_, box.z.min_ },
        { box.x.min_, box.y.min_, box.z.max_ },
        { box.x.min_, box.y.max_, box.z.min_ },
        { box.x.min_, box.y.max_, box.z.max_ },
        { box.x.max_, box.y.min_, box.z.min_ },
        { box.x.max_, box.y.min_, box.z.max_ },
        { box.x.max_, box.y.max_, box.z.min_ },
        { box.x.max_, box.y.max_, box.z.max_ }
    };

    Aabb out;

    // Transform and accumulate corners
    for (int i = 0; i < 8; i++)
        out = Aabb(out, Aabb(M * c[i], M * c[i]));

    return out;
}


// -------------------------------------------------------------
// TYPE ID
// -------------------------------------------------------------
int Instance::TypeId() const {
    return 9999; // unique ID for instances
}

int Instance::ObjectIndex() const {
    return obj_index_;
}

void Instance::SetObjIndex(int i) {
    obj_index_ = i;
}

} // namespace rt::geom

