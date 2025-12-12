#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include <tiny_obj_loader.h>

#include "scene/scene.h"
#include "scene/camera.h"


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

