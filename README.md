# Monte Carlo Ray Tracer

A modern physically-based Monte Carlo ray tracer written in C++.

![Sample Render](assets/outputs-png/sample_render.png)
*Sample scene showcasing spheres with dielectric, specular, and diffuse materials*

## Features

### Core Rendering
- **Monte Carlo Path Tracing**: Physically-based light transport simulation
- **Wavefront Rendering**: Efficient batch-based traversal; future GPU support
- **BVH Acceleration**: SAH-binned BVH for fast intersection testing
- **Multi-threaded CPU Renderer**: OpenMP-based parallel ray evaluation

### Geometry Support
- **Primitive Intersections**: Optimized sphere, quad, and triangle routines
- **Mesh Loading**: OBJ mesh support through TinyObjLoader
- **Instancing**: Scale, rotate (Y-axis), and translate mesh instances

### Material System
- **Lambertian Diffuse** with cosine-weighted sampling
- **Metallic BRDF** with roughness
- **Dielectric Refraction** with Fresnel equations
- **Texture Mapping** from image files
- **Checker and procedural textures**

### In Progress
- **Hybrid GPU Rendering**: GPU BVH traversal and path integration

## Dependencies

- **STB Libraries**: `stb_image.h`, `stb_image_write.h`
- **JSON Library**: [nlohmann/json](https://github.com/nlohmann/json)
- **tinyobjloader**: OBJ geometry import
- **OpenMP**: CPU multi-threading
- **C++17 or later** compiler

## Building

```bash
# Clone the repository
git clone https://github.com/Luke-TS/LTS-PathTracer.git
cd LTS-PathTracer 

# Create build directory
mkdir build && cd build

# Configure and build
cmake ..
cmake --build .


```

## Usage

Scenes are described entirely through JSON. View examples in /scenes or use template below:

```json
{
    "camera": {
        "aspectRatio": 1.5,
        "imageWidth": 800,
        "samplesPerPixel": 250,
        "maxDepth": 120,
        "vfov": 40.0,

        "lookfrom": [5, 8, -10],
        "lookat":   [5, 4, 5],
        "vup":      [0, 1, 0],

        "defocusAngle": 0.0,
        "focusDist": 12.0
    },
    "environment": {
        "type": "cubemap",
        "file": "high-def-sky.png",
        "intensity": 1.0
    },

    "textures": {
        "checker": {
            "type": "checker",
            "scale": 0.32,
            "color1": [0.2, 0.3, 0.1],
            "color2": [0.9, 0.9, 0.9]
        }
    },

    "materials": {
        "red":   { "type": "lambertian", "albedo": [0.75, 0.1, 0.1] },
        "white": { "type": "lambertian", "albedo": [0.73, 0.73, 0.73] },
        "green": { "type": "lambertian", "albedo": [0.12, 0.45, 0.15] },
        "blue":  { "type": "lambertian", "albedo": [0.2, 0.3, 0.8] },
        "metal": { "type": "metal", "albedo": [0.9, 0.9, 0.95], "fuzz": 0.01 },
        "chrome": { "type": "metal", "albedo": [0.95, 0.96, 0.99], "fuzz": 0.0 },
        "glass": { "type": "dielectric", "ior": 1.5 },
        "fogglass": { "type": "dielectric", "ior": 1.33, "absorb": [0.7, 0.85, 1.0] },
        "light": { "type": "diffuse_light", "color": [10,10,10] },
        "fill":  { "type": "diffuse_light", "color": [4,4,4] },
        "checker": {"type": "lambertian", "texture": "checker"}
    },

    "meshes": {
        "dragon": {
            "file": "xyzrgb_dragon.obj",
            "material": "red",
            "normalize": true,
            "unitScale": 1.0
        }
    },

    "objects": [
        {
            "type": "quad",
            "p1": [0,0,0],
            "p2": [0,10,0],
            "p3": [0,10,10],
            "p4": [0,0,10],
            "material": "green"
        },
        {
            "type": "quad",
            "p1": [10,0,10],
            "p2": [10,10,10],
            "p3": [10,10,0],
            "p4": [10,0,0],
            "material": "red"
        },
        {
            "type": "quad",
            "p1": [0,0,0],
            "p2": [10,0,0],
            "p3": [10,0,10],
            "p4": [0,0,10],
            "material": "metal"
        },
        {
            "type": "quad",
            "p1": [0,10,10],
            "p2": [10,10,10],
            "p3": [10,10,0],
            "p4": [0,10,0],
            "material": "white"
        },
        {
            "type": "quad",
            "p1": [0,0,10],
            "p2": [0,10,10],
            "p3": [10,10,10],
            "p4": [10,0,10],
            "material": "checker"
        },

        {
            "type": "quad",
            "p1": [2,9.99,2],
            "p2": [8,9.99,2],
            "p3": [8,9.99,8],
            "p4": [2,9.99,8],
            "material": "light"
        },
        {
            "type": "quad",
            "p1": [0.01,4,2],
            "p2": [0.01,6,2],
            "p3": [0.01,6,4],
            "p4": [0.01,4,4],
            "material": "fill"
        },
        {
            "type": "quad",
            "p1": [9.99,3,6],
            "p2": [9.99,5,6],
            "p3": [9.99,5,8],
            "p4": [9.99,3,8],
            "material": "fill"
        },

        {
            "type": "instance",
            "instanceOf": "dragon",
            "translate": [5, 2.15, 6.0],
            "scale": 7.0
        },

        { "type":"sphere", "center":[2,7,6], "radius":1.2, "material":"glass" },
        { "type":"sphere", "center":[7.0,6.0,5.2], "radius":1.0, "material":"metal" }
    ]
}
```

## Rendering

```bash
./raytracer <scene.json> [output.ext]
```

Support outputs: png, ppm

## Example Renders

> **Note:** Working to implement Next-Event Estimation to reduce noise in Cornell Box render.

| Cornell Box | Textured Mesh |
|-------------|---------------|
| ![TDOD](assets/outputs-png/cornell.png) | ![TODO](outputs-png/mesh.png) |


## References

- *Ray Tracing in One Weekend* series by Peter Shirley
- *Physically Based Rendering: From Theory to Implementation* by Pharr, Jakob, and Humphreys
