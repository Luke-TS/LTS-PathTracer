#pragma once

#include <memory>
#include "geom/hittable.h"
#include "core/mat4.h"

namespace rt::geom {

class Instance : public Hittable {
public:
    Instance(const std::shared_ptr<Hittable>& obj,
             const core::Mat4& M_,
             const core::Mat4& invM_);

    // Core overrides
    bool Hit(const core::Ray& r_world,
             core::Interval ray_t,
             HitRecord& rec) const override;

    Aabb BoundingBox() const override;

    int TypeId() const override;
    int ObjectIndex() const override;
    void SetObjIndex(int i) override;

private:
    std::shared_ptr<Hittable> object;   
    core::Mat4 M;                       
    core::Mat4 invM;                    
    core::Mat4 invM_T;                 
    int obj_index_ = -1;               
};

} // namespace rt::geom

