#include "geometry/bowl_model.hpp"

#include "geometry/blend_weights.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace avm_stitch {

namespace {

// Three-tier blend-weight normalisation shared by both bake paths (kb and
// multi-model): 1) sector-weighted when any visible camera has non-trivial
// raw weight, 2) geometry-weighted (edge proximity) D3 fallback, 3) all-zero
// when nothing passed the feather gate. `edge_ok[j]` marks cameras whose
// pixel for this vertex passed the visibility gates with a non-zero feather.
void NormalizeWeights(int n,
                      const std::vector<double>& raw_weights,
                      const std::vector<char>& edge_ok,
                      const std::vector<double>& edge_factor,
                      BakedVertex& bv) {
    double visible_contrib_sum = 0.0;
    double edge_only_sum = 0.0;
    for (int j = 0; j < n; ++j) {
        if (edge_ok[j]) {
            if (raw_weights[j] > 1e-9) visible_contrib_sum += raw_weights[j] * edge_factor[j];
            edge_only_sum += edge_factor[j];
        }
    }

    if (visible_contrib_sum > 1e-9) {
        // Tier 1: sector-weighted (normal path)
        for (int j = 0; j < n; ++j) {
            if (edge_ok[j] && raw_weights[j] > 1e-9) {
                bv.weight[j] = static_cast<float>(
                    raw_weights[j] * edge_factor[j] / visible_contrib_sum);
            }
        }
    } else if (edge_only_sum > 1e-9) {
        // Tier 2: geometry-weighted (D3 re-fallback — sector model assigned
        // all weight to an invisible camera, so distribute over all
        // geometrically-visible cameras by edge proximity).
        for (int j = 0; j < n; ++j) {
            if (edge_ok[j]) {
                bv.weight[j] = static_cast<float>(edge_factor[j] / edge_only_sum);
            }
        }
    }
    // Tier 3: no visible camera with non-zero feather → all weights stay 0.
}

} // namespace

// ---------------------------------------------------------------------------
// DeriveBowlParams: all bowl parameters from the vehicle's real dimensions.
// One formula for cars and ships alike: pure power laws in the SINGLE scale
// factor s = L/5.6. Every length ∝ s and the curvature coefficient
// k_slop ∝ s^-1.2, so the surface shape is EXACTLY self-similar — a 38x
// longer ship is a 38x larger copy of the calibrated car bowl (grid size,
// opening ratio, wall slope and longitudinal/lateral rim-height isotropy are
// all scale invariant). No per-vehicle presets, no if-branches.
//   r_outter = 12.6 s     round_l = 2.0 s     step = 0.2 s
//   max_z    = 8.6 s      z_exp   = 2.2       k_slop = 0.09 s^-1.2
// Anchor (s = 1): exactly the old calibrated Default — the historically
// healthy car bowl (r_outter=12.6, round_l=2.0, step=0.2, max_z=8.6,
// k_slop=0.09, z_exp=2.2).
// Self-similar structure at every scale (NOT "clipped harder as it grows"):
//   - grid = 2*lround(63) = 126x126 always (r_outter/step == 63);
//   - opening ratio rd*/r_outter = ((max_z/k_slop)^(1/2.2))/r_outter =
//     (95.6)^(1/2.2)/12.6 ≈ 0.63 — the wall climbs to max_z at the same
//     relative radius and clips, keeping the "steep wall + opening"
//     silhouette (it never degenerates into a shallow plate);
//   - longitudinal vs lateral rim heights stay isotropic (~1.04:1) instead
//     of forming a "两侧起墙、船头尾全平" trough (the old sub-linear
//     family: 3 m bow/stern vs 35 m sides at the ship — 11.5:1).
// ---------------------------------------------------------------------------
BowlParams DeriveBowlParams(double car_length, double car_width, double axis_x) {
    BowlParams p;
    p.car_length = car_length;
    p.car_width = car_width;

    // Reference vehicle: the calibrated car anchor (L0=5.6). Every length is
    // a linear multiple of s = L/L0 — equal scaling, so the surface shape is
    // independent of scale.
    constexpr double kRefLength = 5.6;
    const double scale = car_length / kRefLength;

    // r_outter = 12.6 * s: linear growth — a 38x longer ship gets a 38x
    // wider bowl (476 m), keeping the farthest camera (149.8 m) well inside.
    p.r_outter = kCalibratedAnchorRoutter * scale;

    // Flat-region margin and grid resolution scale linearly with the bowl:
    // round_l = 2.0*s (= 0.1587*r_outter) and step = 0.2*s keep
    // r_outter/step = 63 exactly -> the grid stays 126x126 at every scale.
    p.round_l = 2.0 * scale;      // car: 2.0 (old Default, exact)
    p.step = 0.2 * scale;         // car: 0.2

    // Steep-wall constants from the old calibrated Default, self-similar:
    // z_exp fixed 2.2; max_z grows linearly while k_slop shrinks as s^-1.2,
    // so the opening ratio (max_z/k_slop)^(1/2.2)/r_outter stays at the
    // constant 0.63 and the shape is identical at every scale.
    p.z_exp = 2.2;
    p.max_z = 8.6 * scale;                    // car: 8.6, ship: ~325
    p.k_slop = 0.09 * std::pow(scale, -1.2);  // car: 0.09, ship: ~1.15e-3

    p.type = BowlType::BlockBoat;
    p.car_center = cv::Vec2d(axis_x / 2.0, 0.0);
    return p;
}

// Main constructor: mesh generation (BlockBoat grid) followed by bake.
BowlModel::BowlModel(const BowlParams& params,
                     const std::vector<CameraIntrinsics>& intrinsics,
                     const std::vector<CameraExtrinsics>& extrinsics) {
    GenerateBlockBoatMesh(params);
    BakeVertices(intrinsics, extrinsics, params.car_center);
}

// Multi-model constructor: mesh generation followed by the CameraModel bake.
BowlModel::BowlModel(const BowlParams& params, const std::vector<Camera>& cameras) {
    GenerateBlockBoatMesh(params);
    BakeVertices(cameras, params.car_center);
}

// Multi-model constructor over pre-built vertex data (tests / custom geometry).
BowlModel::BowlModel(std::vector<cv::Vec3d> vertices,
                     std::vector<cv::Vec3d> normals,
                     const BowlParams& params, const std::vector<Camera>& cameras,
                     const cv::Vec2d& car_center)
    : vertices_(std::move(vertices)), normals_(std::move(normals)) {
    BakeVertices(cameras, car_center);
    (void)params;
}

BowlModel::BowlModel(std::vector<cv::Vec3d> vertices,
                     std::vector<cv::Vec3d> normals,
                     const std::vector<CameraIntrinsics>& intrinsics,
                     const std::vector<CameraExtrinsics>& extrinsics,
                     const cv::Vec2d& car_center)
    : vertices_(std::move(vertices)), normals_(std::move(normals)) {
    BakeVertices(intrinsics, extrinsics, car_center);
}

// ---------------------------------------------------------------------------
// Shared: distance from point (x, y) to the rounded-rectangle flat region.
// ---------------------------------------------------------------------------
double BowlModel::ComputeRd(double x, double y, const BowlParams& params) {
    // D1: offset by car_center so the flat region is centered at car_center,
    // not at the data origin. car_center=(0,0) recovers old behavior.
    double car_l_half = params.car_length / 2.0;
    double car_w_half = params.car_width / 2.0;
    double rel_x = x - params.car_center[0];
    double rel_y = y - params.car_center[1];

    double dx = std::max(0.0, std::abs(rel_x) - (car_l_half + params.round_l));
    double dy = std::max(0.0, std::abs(rel_y) - (car_w_half + params.round_l));
    return std::sqrt(dx * dx + dy * dy);
}

// ---------------------------------------------------------------------------
// Square grid mesh (create_block_boat_model from avm_renderer).
// ---------------------------------------------------------------------------
void BowlModel::GenerateBlockBoatMesh(const BowlParams& params) {
    const double x_half = params.car_length / 2.0 + params.round_l;
    const double y_half = params.car_width / 2.0 + params.round_l;

    rows_ = cols_ = static_cast<int>(std::lround(params.r_outter / params.step) * 2);

    std::vector<int> grid_to_index((rows_ + 1) * (cols_ + 1), -1);
    int vertex_count = 0;

    // Step 1: generate vertices (square grid, VCS coordinates)
    for (int i = 0; i <= rows_; ++i) {
        for (int j = 0; j <= cols_; ++j) {
            double u = static_cast<double>(i) / rows_ * 2.0 - 1.0;
            double v = static_cast<double>(j) / cols_ * 2.0 - 1.0;
            // D1: grid extent centered at car_center, not at data origin.
            // u,v ∈ [-1,1] → x ∈ [cx-r_outter, cx+r_outter].
            double x = params.car_center[0] + u * params.r_outter;
            double y = params.car_center[1] + v * params.r_outter;

            double rd = ComputeRd(x, y, params);
            if (rd > params.r_outter + 1e-9) continue;

            double z = 0.0;
            if (rd > 0.0) {
                z = params.k_slop * std::pow(rd, params.z_exp);
            }
            if (z > params.max_z) continue;

            vertices_.push_back(cv::Vec3d(x, y, z));
            normals_.push_back(cv::Vec3d(0.0, 0.0, 1.0));
            grid_to_index[i * (cols_ + 1) + j] = vertex_count++;
        }
    }

    // Step 2: recompute normals using finite difference
    ComputeBowlNormals(*this, grid_to_index, params, rows_, cols_);

    // Step 3: build triangle indices (square grid connectivity)
    for (int i = 0; i < rows_; ++i) {
        for (int j = 0; j < cols_; ++j) {
            int p1 = grid_to_index[i * (cols_ + 1) + j];
            int p2 = grid_to_index[i * (cols_ + 1) + (j + 1)];
            int p3 = grid_to_index[(i + 1) * (cols_ + 1) + j];
            int p4 = grid_to_index[(i + 1) * (cols_ + 1) + (j + 1)];
            if (p1 >= 0 && p2 >= 0 && p3 >= 0 && p4 >= 0) {
                triangles_.push_back({p1, p3, p2});
                triangles_.push_back({p2, p3, p4});
            }
        }
    }
}

// Finite-difference normals (verbatim from the former header-only
// implementation; only the `inline` keyword was dropped).
void BowlModel::ComputeBowlNormals(BowlModel& mesh,
                                  const std::vector<int>& grid_to_index,
                                  const BowlParams& params, int rows,
                                  int cols) {
    for (int i = 0; i <= rows; ++i) {
        for (int j = 0; j <= cols; ++j) {
            int idx = grid_to_index[i * (cols + 1) + j];
            if (idx < 0) continue;
            double z = mesh.vertices_[idx][2];
            if (z < 1e-9) {
                mesh.normals_[idx] = cv::Vec3d(0.0, 0.0, 1.0);
                continue;
            }
            double dz_dx = 0.0, dz_dy = 0.0;
            int count_x = 0, count_y = 0;
            // Accumulate one neighbour's height difference; side = +1 for the
            // +i/+j direction, -1 for the -i/-j direction (centred difference).
            auto add_neighbor = [&](int ni, int nj, int side, double& acc, int& count) {
                if (ni < 0 || ni > rows || nj < 0 || nj > cols) return;
                int nidx = grid_to_index[ni * (cols + 1) + nj];
                if (nidx < 0) return;
                acc += side * (mesh.vertices_[nidx][2] - z);
                ++count;
            };
            add_neighbor(i, j - 1, -1, dz_dx, count_x);
            add_neighbor(i, j + 1, +1, dz_dx, count_x);
            add_neighbor(i - 1, j, -1, dz_dy, count_y);
            add_neighbor(i + 1, j, +1, dz_dy, count_y);
            if (count_x > 0) dz_dx /= count_x;
            if (count_y > 0) dz_dy /= count_y;

            cv::Vec3d n(-dz_dx, -dz_dy, 1.0);
            double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
            mesh.normals_[idx] = (len > 1e-9) ? n / len : cv::Vec3d(0.0, 0.0, 1.0);
        }
    }
}

// Per-vertex bake (reads the model's own mesh data via member access).
// Called once at the end of each constructor: the construction invariant
// guarantees a fresh buffer, so no stale-slot clearing is needed.
void BowlModel::BakeVertices(const std::vector<CameraIntrinsics>& intrinsics,
                             const std::vector<CameraExtrinsics>& extrinsics,
                             const cv::Vec2d& car_center) {
    const int N = static_cast<int>(intrinsics.size());
    baked_num_cameras_ = N;
    baked_vertices_ = std::vector<BakedVertex>(vertex_count());

    for (int i = 0; i < vertex_count(); ++i) {
        const cv::Vec3d& P = vertices_[i];
        auto& bv = baked_vertices_[i];

        const auto raw_weights = ComputeBlendWeights(P, extrinsics, car_center);

        // Visibility tracked by an explicit flag (not UV values), since a
        // visible point may legally project to normalized UV (0,0).
        std::vector<bool> visible(N, false);
        std::vector<double> edge_factor(N, 0.0);
        for (int j = 0; j < N; ++j) {
            double u_px, v_px;
            ProjectWorldToFishEye(P, intrinsics[j], extrinsics[j], u_px, v_px);

            // Exclude the fisheye black dead zone: a pixel may be inside the
            // image RECTANGLE but outside the effective lens circle (the
            // corners beyond the disc are black) — sampling it would pollute
            // the seam with black. Require the projected pixel to lie within
            // the fisheye effective circle.
            // D3: IsForwardFacing removed from visibility gating — the hand-written
            // fisheye projection handles all z values correctly, and the fisheye
            // circle check + image rectangle check naturally reject points outside
            // the effective FOV. Points behind the camera (z<0) that project inside
            // the image are now correctly visible.
            visible[j] = u_px >= 0.0 && u_px < intrinsics[j].width
                         && v_px >= 0.0 && v_px < intrinsics[j].height
                         && IsInsideFisheyeCircle(intrinsics[j], u_px, v_px);

            if (visible[j]) {
                bv.uv[j][0] = static_cast<float>(u_px / intrinsics[j].width);
                // OpenCV/T_wc convention: camera y axis points DOWN, so v_px already grows
                // toward the image bottom (projectPoints output is a direct image
                // coordinate, same as solve_extrinsic). No V flip needed here — the old
                // 1.0 - v_px/height flip was a compensation for the y-up convention.
                bv.uv[j][1] = static_cast<float>(v_px / intrinsics[j].height);
                // Feather near the fisheye circle edge: hard gate — edge_factor==0
                // means the pixel falls within the dark vignette zone (measured
                // luminance r220=71, r230=19, r240=0.15) and the camera must be
                // treated as invisible for that vertex.
                edge_factor[j] = FisheyeEdgeFeather(intrinsics[j], u_px, v_px);
                // Gate visibility: unsafe content is invisible. The D3 fallback
                // normalisation only operates on gate-passing cameras, preventing
                // black-rim samples from being amplified to full weight.
                visible[j] = visible[j] && edge_factor[j] > 0.0;
            }
        }

        // Normalize raw blend weights over the visible camera set.
        // Three-tier weight assignment (see NormalizeWeights):
        // 1. Normal: at least one visible camera has non-trivial raw weight →
        //    renormalise over that subset (preserves sector preference).
        // 2. D3 fallback: visible cameras exist but all have negligible raw
        //    weight (sector model assigned weight to an invisible camera) →
        //    distribute proportionally by edge_factor alone (geometry-weighted).
        // 3. No visible cameras with non-zero edge_factor → weights stay 0
        //    (centre car-floor physical blind zone — expected behaviour).
        std::vector<char> edge_ok(N, 0);
        for (int j = 0; j < N; ++j) edge_ok[j] = visible[j] && edge_factor[j] > 0.0;
        NormalizeWeights(N, raw_weights, edge_ok, edge_factor, bv);
    }
}

// ---------------------------------------------------------------------------
// Multi-model bake: identical gating/weight math to the kb overload above;
// projection is delegated to CameraModel::ProjectPoints — one virtual call
// per camera (batched over all vertices), circle/feather gating reuse the
// projection.hpp helpers via a synthesized intrinsics (cx/cy/radius from the
// model; radius 0 -> the auto inscribed-circle branch).
// ---------------------------------------------------------------------------
void BowlModel::BakeVertices(const std::vector<Camera>& cameras,
                             const cv::Vec2d& car_center) {
    const int N = static_cast<int>(cameras.size());
    for (int j = 0; j < N; ++j) {
        if (cameras[j].model == nullptr) {
            throw std::invalid_argument(
                "BowlModel: camera " + std::to_string(j) +
                " has no model (Camera.model is null)");
        }
    }
    baked_num_cameras_ = N;
    baked_vertices_ = std::vector<BakedVertex>(vertex_count());

    // Per camera, per vertex edge factor (0 = invisible / feather-gated).
    // Function-local: the projection pass fills it, the weight pass consumes
    // it — no need to widen the object's mutable state.
    std::vector<std::vector<double>> edge_factors(
        static_cast<size_t>(N), std::vector<double>(vertex_count(), 0.0));

    // Camera-frame point buffers, reused across cameras (batched: one
    // ProjectPoints call per camera — hot-loop constraint, spec D1 3.3).
    std::vector<cv::Vec3d> cam_points(vertex_count());
    std::vector<cv::Vec2d> uv(vertex_count());
    std::vector<uint8_t> valid(vertex_count());

    for (int j = 0; j < N; ++j) {
        const CameraModel& model = *cameras[j].model;
        const CameraExtrinsics& extr = cameras[j].extrinsics;
        for (int i = 0; i < vertex_count(); ++i) {
            cam_points[i] = TransformWorldToCamera(vertices_[i], extr);
        }
        model.ProjectPoints(cam_points, uv, valid);

        // Gating intrinsics: only cx/cy/width/height/fisheye_radius are
        // consumed by IsInsideFisheyeCircle / FisheyeEdgeFeather. A zero
        // radius takes the auto inscribed-circle branch (projection.hpp).
        CameraIntrinsics gate{};
        gate.width = model.width();
        gate.height = model.height();
        gate.cx = model.Cx();
        gate.cy = model.Cy();
        gate.fisheye_radius = model.FisheyeRadius();

        auto& factors = edge_factors[static_cast<size_t>(j)];
        for (int i = 0; i < vertex_count(); ++i) {
            auto& bv = baked_vertices_[i];
            const double u_px = uv[i][0];
            const double v_px = uv[i][1];

            // Rectangle (from out_valid) + circle gate — same criterion and
            // write order as the kb bake path above: UVs are written for
            // rect+circle-visible vertices even when the feather gate later
            // marks the vertex invisible (bit-parity with legacy bake).
            const bool in_circle = valid[i] != 0
                                   && IsInsideFisheyeCircle(gate, u_px, v_px);
            if (in_circle) {
                bv.uv[j][0] = static_cast<float>(u_px / gate.width);
                bv.uv[j][1] = static_cast<float>(v_px / gate.height);
                factors[static_cast<size_t>(i)] =
                    FisheyeEdgeFeather(gate, u_px, v_px);
            }
        }
    }

    // Weight normalisation (three tiers, identical to the kb path) runs per
    // vertex now that visibility/edge factors are known for every camera.
    const auto extrinsics = ExtractExtrinsics(cameras);
    for (int i = 0; i < vertex_count(); ++i) {
        const cv::Vec3d& P = vertices_[i];
        const auto raw_weights = ComputeBlendWeights(P, extrinsics, car_center);

        std::vector<char> edge_ok(N, 0);
        std::vector<double> edge_factor(N, 0.0);
        for (int j = 0; j < N; ++j) {
            edge_factor[j] = edge_factors[static_cast<size_t>(j)][static_cast<size_t>(i)];
            edge_ok[j] = edge_factor[j] > 0.0;
        }
        NormalizeWeights(N, raw_weights, edge_ok, edge_factor, baked_vertices_[i]);
    }
}

std::vector<CameraExtrinsics> BowlModel::ExtractExtrinsics(
    const std::vector<Camera>& cameras) {
    std::vector<CameraExtrinsics> extrs;
    extrs.reserve(cameras.size());
    for (const auto& cam : cameras) extrs.push_back(cam.extrinsics);
    return extrs;
}

bool BowlModel::HasVisibleVertices() const {
    // Same visibility criterion as the former test helper in
    // test_stitch_engine.cpp: a vertex counts as visible when the sum of its
    // camera weights exceeds 1e-9. Since all weights are >= 0, the sum is
    // > 1e-9 exactly when at least one slot is > 1e-9.
    const int n = vertex_count();
    const int cams = baked_num_cameras_;
    for (int i = 0; i < n; ++i) {
        const BakedVertex& bv = baked_vertices_[i];
        double sum = 0.0;
        for (int j = 0; j < cams; ++j) sum += bv.weight[j];
        if (sum > 1e-9) return true;
    }
    return false;
}

} // namespace avm_stitch
