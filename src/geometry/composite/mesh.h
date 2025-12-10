#pragma once

#include <memory>
#include <vector>
#include <array>

#include "hittable.h"
#include "scene/scene.h"
#include "triangle.h"

namespace rt::geom {

class Mesh : public Hittable {
public:
    scene::Scene tris_;

    Mesh(const std::vector<core::Point3>& vertices,
                  const std::vector<std::array<int,3>>& indices,
                  std::shared_ptr<material::Material> mat) {
        for( const auto& face : indices ) {
            auto tri = std::make_shared<Triangle>(
                vertices[face[0]],
                vertices[face[1]],
                vertices[face[2]],
                mat
            );
            tris_.Add(tri);
        }
    }

    virtual bool Hit(const core::Ray& r, core::Interval ray_t, HitRecord& rec) const override {
        return tris_.Hit(r, ray_t, rec);
    }

    virtual Aabb BoundingBox() const override {
        return tris_.BoundingBox();
    }
};

} // namespace rt::geom
