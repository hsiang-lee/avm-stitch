#include "geometry/mesh_bake.hpp"

#include "geometry/blend_weights.hpp"

#include <vector>

namespace avm_stitch {

BakedMesh BakeMesh(const BowlMesh& bowl_mesh,
                   const std::vector<CameraIntrinsics>& intrinsics,
                   const std::vector<CameraExtrinsics>& extrinsics) {
    const int N = static_cast<int>(intrinsics.size());
    BakedMesh baked;
    baked.num_cameras = N;
    baked.vertices.resize(bowl_mesh.vertices.size());

    for (size_t i = 0; i < bowl_mesh.vertices.size(); ++i) {
        const auto& P = bowl_mesh.vertices[i];
        auto& bv = baked.vertices[i];

        const auto raw_weights = ComputeBlendWeights(P, extrinsics);

        // Visibility tracked by an explicit flag (not UV values), since a
        // visible point may legally project to normalized UV (0,0).
        std::vector<bool> visible(N, false);
        std::vector<double> edge_factor(N, 0.0);
        double weight_sum = 0.0;
        for (int j = 0; j < N; ++j) {
            double u_px, v_px;
            ProjectWorldToFishEye(P, intrinsics[j], extrinsics[j], u_px, v_px);

            // Exclude the fisheye black dead zone: a pixel may be inside the
            // image RECTANGLE but outside the effective lens circle (the
            // corners beyond the disc are black) — sampling it would pollute
            // the seam with black. Require the projected pixel to lie within
            // the fisheye effective circle.
            visible[j] = IsForwardFacing(P, extrinsics[j])
                         && u_px >= 0.0 && u_px < intrinsics[j].width
                         && v_px >= 0.0 && v_px < intrinsics[j].height
                         && IsInsideFisheyeCircle(intrinsics[j], u_px, v_px);

            if (visible[j]) {
                bv.uv[j][0] = static_cast<float>(u_px / intrinsics[j].width);
                // OpenCV/T_wc convention: camera y axis points DOWN, so v_px already grows
                // toward the image bottom (projectPoints output is a direct image
                // coordinate, same as solve_extrinsic). No V flip needed here — the old
                // 1.0 - v_px/height flip was a compensation for the y-up convention.
                bv.uv[j][1] = static_cast<float>(v_px / intrinsics[j].height);
                // Feather near the fisheye circle edge so linear filtering does
                // not mix the black dead-zone into the seam blend.
                edge_factor[j] = FisheyeEdgeFeather(intrinsics[j], u_px, v_px);
                weight_sum += raw_weights[j] * edge_factor[j];
            }
        }

        // Normalize raw blend weights over the visible camera set only. A
        // camera whose feather factor is 0 keeps weight 0 and does not
        // participate in the normalization.
        if (weight_sum > 1e-9) {
            for (int j = 0; j < N; ++j) {
                if (visible[j] && raw_weights[j] > 1e-9 && edge_factor[j] > 0.0) {
                    bv.weight[j] = static_cast<float>(
                        raw_weights[j] * edge_factor[j] / weight_sum);
                }
            }
        }
    }

    return baked;
}

} // namespace avm_stitch
