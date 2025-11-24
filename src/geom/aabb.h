#pragma once

#include "core/interval.h"
#include "core/vec3.h"
#include "core/ray.h"

namespace rt::geom {

class Aabb {
public:
    core::Interval x, y ,z;

    // intervals are empty by default
    Aabb();

    // initialize intervals
    Aabb(const core::Interval& x, const core::Interval& y, const core::Interval& z);

    // create box from 2 points
    Aabb(const core::Point3& a, const core::Point3& b);

    // create box overlapping 2 boxes
    Aabb(const Aabb& a, const Aabb& b); 

    // Expand an existing box to include a point
    Aabb(const Aabb& box, const core::Vec3& p); 

    // 0:x 1:y 2:z
    const core::Interval& AxisInterval(int n) const; 

    core::Vec3 Min() const;

    core::Vec3 Max() const;

    core::Vec3 Center() const;

    // RETURN 0, 1, or 2
    int LongestAxis() const; 

    // Needed for SAH
    double SurfaceArea() const; 

    bool Hit(const core::Ray& r, core::Interval ray_t) const; 
};

} // namespace rt::geom
