#include "geom/quad.h"
#include "scene/scene_loader.h"

#include <fstream>
#include <stdexcept>
#include <tiny_obj_loader.h>

#include "geom/sphere.h"
#include "geom/quad.h"
#include "geom/triangle.h"
#include "geom/bvh.h"
#include "geom/instance.h"

#include "material/material.h"
#include "material/texture.h"

using json = nlohmann::json;

namespace rt::scene {

// global mesh cache
static std::unordered_map<std::string, std::shared_ptr<geom::Hittable>> mesh_cache;

static core::Color readColor(const json& j) {
    return core::Color(j[0], j[1], j[2]);
}

static core::Vec3 readVec3(const json& j) {
    return core::Vec3(j[0], j[1], j[2]);
}

static core::Point3 readPoint3(const json& j) {
    return core::Point3(j[0], j[1], j[2]);
}

static std::shared_ptr<material::Material>
makeMaterial(const std::string& name, const json& m,
             const std::unordered_map<std::string, 
                 std::shared_ptr<material::Texture>>& textures)
{
    std::string type = m.at("type");

    if (type == "lambertian") {
        if (m.contains("albedo"))
            return std::make_shared<material::Lambertian>(readColor(m["albedo"]));
        if (m.contains("texture"))
            return std::make_shared<material::Lambertian>(textures.at(m["texture"]));
    }

    if (type == "metal") {
        return std::make_shared<material::Metal>(
            readColor(m["albedo"]),
            m.value("fuzz", 0.0)
        );
    }

    if (type == "dielectric") {
        return std::make_shared<material::Dielectric>(m.at("ior"));
    }

    if (type == "diffuse_light") {
        return std::make_shared<material::DiffuseLight>(readColor(m["color"]));
    }

    throw std::runtime_error("Unknown material: " + type);
}

static std::shared_ptr<material::Texture>
makeTexture(const std::string& name, const json& t)
{
    std::string type = t.at("type");

    if (type == "image") {
        return std::make_shared<material::ImageTexture>(t.at("path"));
    }

    if (type == "checker") {
        return std::make_shared<material::CheckerTexture>(
            t.at("scale"),
            readColor(t.at("color1")),
            readColor(t.at("color2"))
        );
    }

    throw std::runtime_error("Unknown texture: " + type);
}

// mesh loader
static std::shared_ptr<geom::Hittable>
loadMeshOBJ(const std::string& path,
            std::shared_ptr<material::Material> mat,
            double uniform_scale,
            bool normalize)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> unused;

    std::string warn, err;

    std::string fullpath = path;
#ifdef MODEL_DIR
    fullpath = std::string(MODEL_DIR) + "/" + path;
#endif

    if (!tinyobj::LoadObj(&attrib, &shapes, &unused, &warn, &err, fullpath.c_str())) {
        throw std::runtime_error("Failed OBJ: " + fullpath + " " + warn + " " + err);
    }

    // bounding box for normalization
    core::Vec3 bb_min(1e30,1e30,1e30);
    core::Vec3 bb_max(-1e30,-1e30,-1e30);

    struct TriRaw { core::Vec3 v0, v1, v2; };
    std::vector<TriRaw> raw;

    for (const auto& shape : shapes) {
        size_t idx_offset = 0;

        for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); f++) {
            int n = shape.mesh.num_face_vertices[f];
            if (n != 3) continue;

            core::Vec3 v[3];

            for (int i=0; i<3; i++) {
                auto idx = shape.mesh.indices[idx_offset + i];

                float x = attrib.vertices[3*idx.vertex_index + 0];
                float y = attrib.vertices[3*idx.vertex_index + 1];
                float z = attrib.vertices[3*idx.vertex_index + 2];

                core::Vec3 p(x, y, z);

                bb_min = core::Vec3(
                    std::min(bb_min.x(), p.x()),
                    std::min(bb_min.y(), p.y()),
                    std::min(bb_min.z(), p.z())
                );

                bb_max = core::Vec3(
                    std::max(bb_max.x(), p.x()),
                    std::max(bb_max.y(), p.y()),
                    std::max(bb_max.z(), p.z())
                );

                v[i] = p;
            }

            raw.push_back({v[0], v[1], v[2]});
            idx_offset += 3;
        }
    }

    // Normalize to unit cube
    if (normalize) {
        core::Vec3 ext = bb_max - bb_min;
        double maxd = std::max({ext.x(), ext.y(), ext.z()});
        core::Vec3 center = (bb_min + bb_max) * 0.5;

        double s = 1.0 / maxd;

        for (auto& t : raw) {
            t.v0 = (t.v0 - center) * s;
            t.v1 = (t.v1 - center) * s;
            t.v2 = (t.v2 - center) * s;
        }
    }

    // Apply uniform scale
    for (auto& t : raw) {
        t.v0 *= uniform_scale;
        t.v1 *= uniform_scale;
        t.v2 *= uniform_scale;
    }

    // Convert into triangles
    std::vector<std::shared_ptr<geom::Hittable>> tris;
    tris.reserve(raw.size());

    for (auto& t : raw)
        tris.push_back(std::make_shared<geom::Triangle>(t.v0, t.v1, t.v2, mat));

    return std::make_shared<geom::Bvh>(tris);
}


// transform helpers
static core::Mat4 ComposeTransform(const core::Vec3& translate,
                                   double rotateY_deg,
                                   double scale)
{
    core::Mat4 T = core::Mat4::Translate(translate);
    core::Mat4 R = core::Mat4::RotateY(core::DegreesToRadians(rotateY_deg));
    core::Mat4 S = core::Mat4::Scale(scale);
    return T * R * S;
}

static std::shared_ptr<geom::Hittable>
makeObject(const json& obj,
           const std::unordered_map<std::string, 
               std::shared_ptr<material::Material>>& mats)
{
    std::string type = obj.at("type");

    if (type == "sphere") {
        core::Point3 c = readPoint3(obj.at("center"));
        double r = obj.at("radius");
        auto m = mats.at(obj.at("material"));
        return std::make_shared<geom::Sphere>(c, r, m);
    }

    if (type == "quad") {
        return std::make_shared<geom::Quad>(
            obj.value("p1", std::vector<double>{0,0,0}),
            obj.value("p2", std::vector<double>{0,0,0}),
            obj.value("p3", std::vector<double>{0,0,0}),
            obj.value("p4", std::vector<double>{0,0,0}),
            mats.at(obj.at("material"))
        );
    }

    if (type == "instance") {
        std::string key = obj.at("instanceOf");
        auto base = mesh_cache.at(key);

        double scale = obj.value("scale", 1.0);
        core::Vec3 translate = obj.value("translate", std::vector<double>{0,0,0});
        double rotateY = obj.value("rotateY", 0.0);

        core::Mat4 M = ComposeTransform(translate, rotateY, scale);
        core::Mat4 invM = M.Inverse();

        return std::make_shared<geom::Instance>(base, M, invM);
    }

    throw std::runtime_error("Unknown object type: " + type);
}

SceneLoadResult SceneLoader::LoadFromJSON(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("Cannot open scene file: " + path);

    json j = json::parse(f);

    CameraConfig cam_cfg = parseCamera(j.at("camera"));

    // TEXTURES
    std::unordered_map<std::string, std::shared_ptr<material::Texture>> textures;
    if (j.contains("textures")) {
        for (auto& [name, def] : j["textures"].items()) {
            textures[name] = makeTexture(name, def);
        }
    }

    // MATERIALS
    std::unordered_map<std::string, std::shared_ptr<material::Material>> mats;
    for (auto& [name, def] : j.at("materials").items()) {
        mats[name] = makeMaterial(name, def, textures);
    }

    // MESHES
    if (j.contains("meshes")) {
        for (auto& [name, def] : j["meshes"].items()) {
            mesh_cache[name] = loadMeshOBJ(
                def.at("file"),
                mats.at(def.at("material")),
                def.value("scale", 1.0),
                def.value("normalize", false)
            );
        }
    }

    // OBJECTS
    Scene world;
    for (auto& obj : j.at("objects")) {
        world.Add(makeObject(obj, mats));
    }

    // TOP-LEVEL BVH
    Scene final;
    final.Add(std::make_shared<geom::Bvh>(world));

    return { final, cam_cfg };
}

} // namespace rt::scene

