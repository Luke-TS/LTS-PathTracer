#pragma once

#include "core/types/ray.h"

#include "rendering/framebuffer.h"
#include "integrators/ray_integrator.h"
#include "integrators/pixel_state.h"
#include "integrators/ray_state.h"

#include "scene/scene.h"
#include "scene/camera.h"
#include <omp.h>

namespace rt::scene {
class Scene;
class Camera;
}

namespace rt::integrator {
class RayIntegrator;
}

namespace rt::renderer {

class WavefrontRenderer {
public:
    WavefrontRenderer(
        const scene::Scene&      world,
        const scene::Camera&     cam,
        integrator::RayIntegrator& integrator,
        int max_depth   = 10,
        int max_samples = 128,
        int batch_size  = 8192
    );

    Framebuffer Render();   // Only declaration here

private:
    const scene::Scene&   world;
    const scene::Camera&  cam;
    integrator::RayIntegrator& integrator;

    int max_depth;
    int max_ssp;
    int batch_size;

    static rt::core::Color background(const rt::core::Ray& r);
};

} // namespace rt::renderer
