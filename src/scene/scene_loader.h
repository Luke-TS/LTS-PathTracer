#pragma once
#include <string>
#include <unordered_map>
#include <memory>
#include <nlohmann/json.hpp>

#include "scene/scene.h"
#include "scene/camera.h"
#include "geom/hittable.h"
#include "geom/sphere.h"
#include "geom/rect.h"
#include "geom/bvh.h"
#include "material/material.h"

namespace rt::scene {

struct SceneLoadResult {
    Scene world;
    CameraConfig camera;
};

class SceneLoader {
public:
    static SceneLoadResult LoadFromJSON(const std::string& path);
};

} // namespace rt::scene

