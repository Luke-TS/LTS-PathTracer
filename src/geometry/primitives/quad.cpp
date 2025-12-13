#include "geometry/primitives/quad.h"
#include "core/math/math_utils.h"

namespace rt::geom {

Quad::Quad(const core::Point3& a,
           const core::Point3& b,
           const core::Point3& c,
           const core::Point3& d,
           std::shared_ptr<material::Material> mat)
    : a_(a), b_(b), c_(c), d_(d), mat_(mat)
{
    // Compute plane normal from first three vertices
    core::Vec3 ab = b - a;
    core::Vec3 ac = c - a;
    normal_ = core::Normalize(core::Cross(ab, ac));
    D_ = core::Dot(normal_, a);
    
    // Set up basis vectors for point-in-quad test
    u_ = b - a;
    v_ = d - a;
    
    // Compute area (for sampling if needed later)
    area_ = core::Cross(u_, v_).length();
    
    // Build bounding box
    double min_x = std::min(std::min(a[0], b[0]), std::min(c[0], d[0]));
    double min_y = std::min(std::min(a[1], b[1]), std::min(c[1], d[1]));
    double min_z = std::min(std::min(a[2], b[2]), std::min(c[2], d[2]));

    double max_x = std::max(std::max(a[0], b[0]), std::max(c[0], d[0]));
    double max_y = std::max(std::max(a[1], b[1]), std::max(c[1], d[1]));
    double max_z = std::max(std::max(a[2], b[2]), std::max(c[2], d[2]));

    core::Point3 min_pt(min_x, min_y, min_z);
    core::Point3 max_pt(max_x, max_y, max_z);
    
    // Pad the bbox slightly to avoid numerical issues
    const double delta = 0.0001;
    bbox_ = Aabb(min_pt - core::Vec3(delta, delta, delta),
                 max_pt + core::Vec3(delta, delta, delta));
}

bool Quad::Hit(const core::Ray& r, core::Interval t_interval, HitRecord& rec) const {
    // Compute ray-plane intersection
    double denom = core::Dot(normal_, r.direction());
    
    // Ray parallel to plane?
    if (std::abs(denom) < 1e-8) {
        return false;
    }
    
    // Solve for t: dot(normal, origin + t*dir) = D
    double t = (D_ - core::Dot(normal_, r.origin())) / denom;
    
    // Check if t is in valid range
    if (!t_interval.Contains(t)) {
        return false;
    }
    
    // Compute hit point
    core::Point3 p = r.at(t);
    
    // Check if point is inside the quad using parametric coordinates
    // Express p in terms of a, u, v: p = a + alpha*u + beta*v
    core::Vec3 ap = p - a_;
    
    // Solve for alpha and beta
    // This is a 2D point-in-parallelogram test projected onto the quad's plane
    double u_dot_u = core::Dot(u_, u_);
    double u_dot_v = core::Dot(u_, v_);
    double v_dot_v = core::Dot(v_, v_);
    double ap_dot_u = core::Dot(ap, u_);
    double ap_dot_v = core::Dot(ap, v_);
    
    double denom_param = u_dot_u * v_dot_v - u_dot_v * u_dot_v;
    
    if (std::abs(denom_param) < 1e-8) {
        return false;
    }
    
    double alpha = (v_dot_v * ap_dot_u - u_dot_v * ap_dot_v) / denom_param;
    double beta = (u_dot_u * ap_dot_v - u_dot_v * ap_dot_u) / denom_param;
    
    // Check if point is inside the parallelogram defined by a, u, v
    if (alpha < 0.0 || alpha > 1.0 || beta < 0.0 || beta > 1.0) {
        return false;
    }
    
    // We have a valid hit!
    rec.t = t;
    rec.p = p;
    rec.mat = mat_;
    rec.front_face = denom < 0;  // Ray hits from front if denom is negative
    rec.normal = rec.front_face ? normal_ : -normal_;

    // Set UV coordinates (can be used for texturing)
    rec.u = alpha;
    rec.v = beta;
    
    return true;
}

Aabb Quad::BoundingBox() const {
    return bbox_;
}

} // namespace rt::geom
