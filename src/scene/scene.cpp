#include "scene/scene.h"

namespace rt::scene {

Scene::Scene() 
    : bbox_(geom::Aabb()) {}

Scene::Scene(std::shared_ptr<geom::Hittable> object)
    : bbox_(geom::Aabb()) 
{
    Add(object);
}

void Scene::Add(const std::shared_ptr<geom::Hittable>& object) {
    if (!object) return;

    objects_.push_back(object);

    // Expand scene bounding box to include object
    bbox_ = geom::Aabb(bbox_, object->BoundingBox());
}

void Scene::Clear() {
    objects_.clear();
    bbox_ = geom::Aabb();   // reset to empty
}

const std::vector<std::shared_ptr<geom::Hittable>>& Scene::Objects() const {
    return objects_;
}

bool Scene::Hit(const core::Ray& r, core::Interval ray_t, geom::HitRecord& rec) const {
    geom::HitRecord temp_rec;
    bool hit_anything = false;
    auto closest_so_far = ray_t.max_;

    for (const auto& object : objects_) {
        if (object->Hit(r, core::Interval(ray_t.min_, closest_so_far), temp_rec)) {
            hit_anything = true;
            closest_so_far = temp_rec.t;
            rec = temp_rec;
        }
    }

    return hit_anything;
}

geom::Aabb Scene::BoundingBox() const {
    return bbox_;
}

int Scene::TypeId() const {
    return -1;
}

int Scene::ObjectIndex() const {
    return -1;
}

void Scene::SetObjIndex(int) {
    // Scene has no GPU index
}

} // namespace rt::scene
