#pragma once

// Per-vertex mesh bake: UV + blend weights per camera slot (pure math).
// Interface only — implementation in mesh_bake.cpp.

#include "geometry/bowl_model.hpp"
#include "geometry/projection.hpp"

#include <vector>

namespace avm_stitch {

// Fixed 8 camera slots (N <= 8). Unused slots stay zero.
struct BakedVertex {
    float uv[8][2];     // normalized [0,1], unused slots = 0
    float weight[8];    // normalized over visible cameras, unused slots = 0

    BakedVertex() {
        for (int j = 0; j < 8; ++j) {
            uv[j][0] = uv[j][1] = 0.0f;
            weight[j] = 0.0f;
        }
    }
};

// Per-vertex baked UV + blend weights. Immutable data holder: read via the
// const getters (zero-copy).
class BakedMesh {
public:
    /// Bake per-vertex UV coordinates and blend weights.
    /// @param bowl_mesh   generated bowl mesh (VCS coordinates)
    /// @param intrinsics  camera intrinsics (size N)
    /// @param extrinsics  camera extrinsics T_wc (size N, must match intrinsics)
    BakedMesh(const BowlMesh& bowl_mesh,
              const std::vector<CameraIntrinsics>& intrinsics,
              const std::vector<CameraExtrinsics>& extrinsics);

    const std::vector<BakedVertex>& vertices() const { return vertices_; }
    int num_cameras() const { return num_cameras_; }

private:
    std::vector<BakedVertex> vertices_;
    int num_cameras_ = 0;
};

} // namespace avm_stitch
