#include "geom/cuboid.h"

namespace rt::geom {

// Cuboid object specified using two points
//
// Uses 6 quad objects internally which each use 2 triangles
// for a total of 12 triangles in the cuboid
Cuboid::Cuboid(const core::Point3& min_pt,
               const core::Point3& max_pt,
               std::shared_ptr<material::Material> mat)
{
    // Store bounding box
    bbox_ = Aabb(min_pt, max_pt);

    // Extract coordinates
    double x0 = min_pt.x(); double y0 = min_pt.y(); double z0 = min_pt.z();
    double x1 = max_pt.x(); double y1 = max_pt.y(); double z1 = max_pt.z();

    // 8 corners of the box
    core::Point3 p000(x0,y0,z0);
    core::Point3 p100(x1,y0,z0);
    core::Point3 p010(x0,y1,z0);
    core::Point3 p110(x1,y1,z0);

    core::Point3 p001(x0,y0,z1);
    core::Point3 p101(x1,y0,z1);
    core::Point3 p011(x0,y1,z1);
    core::Point3 p111(x1,y1,z1);

    // Each face = 2 triangles
    auto add_face = [&](const core::Point3& a,
                        const core::Point3& b,
                        const core::Point3& c,
                        const core::Point3& d)
    {
        faces_.push_back(std::make_shared<Triangle>(a,b,c,mat));
        faces_.push_back(std::make_shared<Triangle>(a,c,d,mat));
    };

    // +X face
    add_face(p100, p110, p111, p101);

    // -X face
    add_face(p000, p001, p011, p010);

    // +Y face
    add_face(p010, p011, p111, p110);

    // -Y face
    add_face(p000, p100, p101, p001);

    // +Z face
    add_face(p001, p101, p111, p011);

    // -Z face
    add_face(p000, p010, p110, p100);
}

bool Cuboid::Hit(const core::Ray& r, core::Interval ray_t, HitRecord& rec) const {
    bool hit_any = false;
    double closest = ray_t.max_;

    HitRecord tmp;
    for (auto& t : faces_) {
        if (t->Hit(r, core::Interval(ray_t.min_, closest), tmp)) {
            hit_any = true;
            closest = tmp.t;
            rec = tmp;
        }
    }

    return hit_any;
}

} // namespace rt::geom
