#include "scene/camera.h"
#include "scene/scene.h"
#include "core/timer.h"
#include "renderer/wavefront.h"
#include "integrator/cpu_ray_integrator.h"
#include "scene/scene_loader.h"

#include <iomanip>
#include <iostream>
#include <ostream>

using namespace rt;

int main(int argc, char** argv) {
    core::Timer clock;
    clock.reset();

    std::string scene_file = "scene.json";
    if (argc > 1) {
        scene_file = argv[1];
    }

    std::clog << "Loading scene: " << scene_file << "\n";

    scene::SceneLoadResult loaded;
    try {
        loaded = scene::SceneLoader::LoadFromJSON(scene_file);
    }
    catch (const std::exception& e) {
        std::cerr << "Error loading scene: " << e.what() << "\n";
        return 1;
    }

    // Unpack scene + camera
    rt::scene::Scene world = loaded.world;
    rt::scene::CameraConfig cam_cfg = loaded.camera;

    // Set up camera
    rt::scene::Camera cam;
    cam.SetFromConfig(cam_cfg);

    // Set up integrators & renderer
    integrator::CPURayIntegrator integrator(&world);

    int samples = cam.samples_per_pixel_;
    int max_depth = cam.max_depth_;

    renderer::WavefrontRenderer renderer(
        world,
        cam,
        integrator,
        max_depth,
        samples,
        2 * 8192 // wavefront batch size
    );

    // Render
    renderer.Render();

    std::clog << "Runtime: " 
              << std::setprecision(2) 
              << clock.elapsed() 
              << "s\n";

    return 0;
}
