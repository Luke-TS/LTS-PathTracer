#include <iostream>
#include <iomanip>
#include "core/timer.h"
#include "integrator/cpu_ray_integrator.h"
#include "scene/scene_loader.h"
#include "scene/camera.h"
#include "renderer/wavefront.h"
#include "renderer/framebuffer.h"

using namespace rt;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::clog << "Usage: ./ray_tracer <scene.json> [output_file.ext]\n"
                  << "  Example: ./ray_tracer scenes/cornell.json output.png\n"
                  << "  Supported formats: .png .ppm\n";
        return 1;
    }

    std::string scene_file  = argv[1];
    std::string output_file = "output.png";      // default

    if (argc >= 3)
        output_file = argv[2];

    std::clog << "Loading scene: " << scene_file << "\n";
    std::clog << "Output file:   " << output_file << "\n";

    scene::SceneLoadResult loaded;
    try {
        loaded = scene::SceneLoader::LoadFromJSON(scene_file);
    }
    catch (const std::exception& e) {
        std::cerr << "Error loading scene: " << e.what() << "\n";
        return 1;
    }

    // Scene + camera
    rt::scene::Scene world = loaded.world;
    rt::scene::CameraConfig cam_cfg = loaded.camera;

    rt::scene::Camera cam;
    cam.SetFromConfig(cam_cfg);

    core::Timer clock;
    clock.reset();

    integrator::CPURayIntegrator integrator(&world);

    renderer::WavefrontRenderer renderer(
        world,
        cam,
        integrator,
        cam.max_depth_,
        cam.samples_per_pixel_,
        2 * 8192  // wavefront batch size
    );

    std::clog << "Rendering...\n";

    renderer::Framebuffer out = renderer.Render();

    out.Save(output_file);

    std::clog << "Saved: " << output_file << "\n";
    std::clog << "Runtime: "
              << std::setprecision(2)
              << clock.elapsed() << "s\n";

    return 0;
}

