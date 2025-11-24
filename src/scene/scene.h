#pragma once

#include <memory>
#include <vector>

#include "core/ray.h"
#include "core/interval.h"

#include "geom/aabb.h"
#include "geom/hittable.h"

namespace rt::scene {

class Scene : public geom::Hittable {
public:
    Scene();
    explicit Scene(std::shared_ptr<geom::Hittable> object);

    void Add(const std::shared_ptr<geom::Hittable>& object);
    void Clear();

    const std::vector<std::shared_ptr<geom::Hittable>>& Objects() const;

    // Hittable interface
    bool Hit(const core::Ray& r, core::Interval ray_t, geom::HitRecord& rec) const override;
    geom::Aabb BoundingBox() const override;

    int TypeId() const override;
    int ObjectIndex() const override;
    void SetObjIndex(int) override;

private:
    std::vector<std::shared_ptr<geom::Hittable>> objects_;
    geom::Aabb bbox_;
};

} // namespace rt::scene
