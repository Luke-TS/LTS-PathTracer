#pragma once

#include <vector>
#include <memory>
#include <algorithm>
#include <limits>

#include "core/ray.h"
#include "core/interval.h"

#include "scene/scene.h"

#include "hittable.h"
#include "aabb.h"
#include "mesh.h"

namespace rt::geom {

/// Node layout usable on both CPU and GPU
struct BvhNodeGPU {
    Aabb     bbox;
    uint32_t left;    // internal: index of left child; leaf: first primitive index
    uint32_t right;   // internal: index of right child; leaf: primitive count
    uint32_t isLeaf;  // 1 = leaf, 0 = internal
};

/// SAH-built, flattened BVH that is a Hittable itself.
class Bvh : public Hittable {
  public:
    // Construct from a Scene
    Bvh(scene::Scene& scene);

    // Construct from a mesh (triangle list)
    Bvh(Mesh& mesh);

    // Construct from explicit list of objects
    Bvh(std::vector<std::shared_ptr<Hittable>>& objects);

    /// Ray intersection (CPU traversal)
    bool Hit(const core::Ray& r, core::Interval ray_t, HitRecord& rec) const override;

    Aabb BoundingBox() const override; 

    // ignore: primitive-only functions (match your old BvhNode)
    int TypeId() const override { return -1; }
    int ObjectIndex() const override { return -1; }
    void SetObjIndex(int) override {}

    // ==== GPU-facing accessors ====

    const std::vector<BvhNodeGPU>& Nodes() const;
    const std::vector<int>&        PrimIndices() const;
    const std::vector<std::shared_ptr<Hittable>>& Primatives() const;

  private:
    // === Internal build structures ===

    struct BuildNode {
        Aabb bounds;
        int  firstPrim = 0;  // offset in prim_indices_
        int  primCount = 0;  // >0 => leaf
        std::unique_ptr<BuildNode> left;
        std::unique_ptr<BuildNode> right;

        bool IsLeaf() const { return primCount > 0; }
    };

    std::vector<std::shared_ptr<Hittable>> primitives_;   // actual geometry
    std::vector<int>    prim_indices_;    // index remapping
    std::vector<Aabb>   prim_bounds_;
    std::vector<core::Vec3>   prim_centroids_;

    std::vector<BvhNodeGPU> nodes_;       // flattened BVH
    int root_index_ = -1;

    static constexpr int   MAX_LEAF_SIZE = 4;
    static constexpr int   BIN_COUNT     = 16;
    static constexpr float TRAVERSAL_COST = 1.0f;
    static constexpr float INTERSECTION_COST = 1.0f;

    // === SAH build ===

    std::unique_ptr<BuildNode> BuildSah(int start, int end); 

    // Flatten build nodes into linear array of BvhNodeGPU
    int Flatten(const BuildNode& bnode); 
};

} // namespace rt::geom
