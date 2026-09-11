#include "geometry/bowl_model.hpp"

#include "geometry/blend_weights.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>

namespace avm_stitch {

// Main constructor: mesh generation (BlockBoat or BowlBoat) followed by bake.
BowlModel::BowlModel(const BowlParams& params,
                     const std::vector<CameraIntrinsics>& intrinsics,
                     const std::vector<CameraExtrinsics>& extrinsics) {
    if (params.type == BowlType::BowlBoat) {
        GenerateBowlBoatMesh(params);
    } else {
        GenerateBlockBoatMesh(params);
    }
    BakeVertices(intrinsics, extrinsics, params.car_center);
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
// BowlBoat rd: 9-region distance to car rectangle (S1-S9).
// Matches avm_renderer create_bowl_boat_model.
// ---------------------------------------------------------------------------
double BowlModel::ComputeRdBowlBoat(double x, double y, const BowlParams& params) {
    // D1: offset by car_center so the 9-region car rectangle is centered at
    // car_center, not at the data origin.
    double car_l_half = params.car_length / 2.0;
    double car_w_half = params.car_width / 2.0;
    double plane_x = car_l_half;
    double plane_y = car_w_half;
    double rel_x = x - params.car_center[0];
    double rel_y = y - params.car_center[1];

    if (rel_x > plane_x) {
        if (rel_y > plane_y) {
            // S1: front-left corner
            return std::sqrt((rel_y - car_w_half) * (rel_y - car_w_half)
                           + (rel_x - car_l_half) * (rel_x - car_l_half));
        } else if (rel_y > -plane_y) {
            // S2: front side
            return std::abs(rel_x - car_l_half);
        } else {
            // S3: front-right corner
            return std::sqrt((rel_y + car_w_half) * (rel_y + car_w_half)
                           + (rel_x - car_l_half) * (rel_x - car_l_half));
        }
    } else if (rel_x >= -plane_x) {
        if (rel_y > plane_y) {
            // S8: left side
            return std::abs(rel_y - car_w_half);
        } else if (rel_y >= -plane_y) {
            // S9: center (inside car rectangle)
            return 0.0;
        } else {
            // S4: right side
            return std::abs(rel_y + car_w_half);
        }
    } else {
        if (rel_y > plane_y) {
            // S7: back-left corner
            return std::sqrt((rel_y - car_w_half) * (rel_y - car_w_half)
                           + (rel_x + car_l_half) * (rel_x + car_l_half));
        } else if (rel_y > -plane_y) {
            // S6: back side
            return std::abs(rel_x + car_l_half);
        } else {
            // S5: back-right corner
            return std::sqrt((rel_y + car_w_half) * (rel_y + car_w_half)
                           + (rel_x + car_l_half) * (rel_x + car_l_half));
        }
    }
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

// ---------------------------------------------------------------------------
// Polar coordinate mesh (create_bowl_boat_model from avm_renderer).
// ---------------------------------------------------------------------------
void BowlModel::GenerateBowlBoatMesh(const BowlParams& params) {
    const int circle_num = std::max(4, static_cast<int>(params.r_outter / params.step));
    const int circle_segment_num = std::max(8, circle_num * 2);
    rows_ = circle_num;
    cols_ = circle_segment_num;

    std::vector<int> grid_to_index((rows_ + 1) * (cols_ + 1), -1);
    int vertex_count = 0;

    // D1: polar grid centered at car_center, not at data origin.
    {
        vertices_.push_back(cv::Vec3d(params.car_center[0], params.car_center[1], 0.0));
        normals_.push_back(cv::Vec3d(0.0, 0.0, 1.0));
        grid_to_index[0] = vertex_count++;
    }

    // Rings from center outward
    for (int circle_n = 1; circle_n <= circle_num; ++circle_n) {
        double r = params.step * circle_n;
        if (circle_n == circle_num) r = params.r_outter;

        for (int seg = 0; seg <= circle_segment_num; ++seg) {
            double angle = seg * 2.0 * M_PI / circle_segment_num;
            // D1: polar circle centered at car_center.
            double x = params.car_center[0] + r * std::sin(angle);
            double y = params.car_center[1] + r * std::cos(angle);

            // 9-region rd (BowlBoat specific)
            double rd = ComputeRdBowlBoat(x, y, params);

            // z = k_slop * pow(rd - L, z_exp) when rd > L
            double z = 0.0;
            double z_r = 0.0;
            if (rd > params.round_l) {
                z_r = rd - params.round_l;
                z = params.k_slop * std::pow(z_r, params.z_exp);
            }
            if (z > params.max_z) continue;

            vertices_.push_back(cv::Vec3d(x, y, z));

            // Normal: flat → quadrant normal, curved → analytical (matching avm_renderer)
            double normal_x, normal_y, normal_z;
            if (rd <= params.round_l || z <= 1e-8) {
                // Quadrant normal (matching reference)
                normal_x = (x > 0 && y > 0) ? 1.0 : 0.0;
                normal_y = (x < 0 && y > 0) ? 1.0 : 0.0;
                normal_z = (x <= 0 && y < 0) ? 1.0 : 0.0;
            } else {
                // Analytical normal for curved surface
                normal_x = -std::sin(angle);
                normal_y = -std::cos(angle);
                normal_z = -(-1.0 / (params.z_exp * z_r));  // 法线朝内
                double len = std::sqrt(normal_x*normal_x + normal_y*normal_y + normal_z*normal_z);
                if (len > 1e-9) {
                    normal_x /= len;
                    normal_y /= len;
                    normal_z /= len;
                }
            }
            normals_.push_back(cv::Vec3d(normal_x, normal_y, normal_z));

            grid_to_index[circle_n * (cols_ + 1) + seg] = vertex_count++;
        }
    }

    // Origin to first ring (fan) — matching avm_renderer
    for (int seg = 0; seg < circle_segment_num; ++seg) {
        int origin = 0;
        int first_ring = 1 + seg;
        int first_ring_next = 1 + ((seg + 1) % circle_segment_num);
        triangles_.push_back({origin, first_ring_next, first_ring});
    }

    // Ring-to-ring connections — matching avm_renderer
    for (int r = 1; r < circle_num; ++r) {
        for (int s = 0; s <= circle_segment_num; ++s) {
            int p1 = grid_to_index[r * (cols_ + 1) + s];
            int p2 = grid_to_index[r * (cols_ + 1) + (s + 1) % (cols_ + 1)];
            int p3 = grid_to_index[(r + 1) * (cols_ + 1) + s];
            int p4 = grid_to_index[(r + 1) * (cols_ + 1) + (s + 1) % (cols_ + 1)];
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
        // Three-tier weight assignment:
        // 1. Normal: at least one visible camera has non-trivial raw weight →
        //    renormalise over that subset (preserves sector preference).
        // 2. D3 fallback: visible cameras exist but all have negligible raw
        //    weight (sector model assigned weight to an invisible camera) →
        //    distribute proportionally by edge_factor alone (geometry-weighted).
        // 3. No visible cameras with non-zero edge_factor → weights stay 0
        //    (centre car-floor physical blind zone — expected behaviour).
        double visible_contrib_sum = 0.0;
        double edge_only_sum = 0.0;
        for (int j = 0; j < N; ++j) {
            if (visible[j] && raw_weights[j] > 1e-9 && edge_factor[j] > 0.0) {
                visible_contrib_sum += raw_weights[j] * edge_factor[j];
            }
            if (visible[j] && edge_factor[j] > 0.0) {
                edge_only_sum += edge_factor[j];
            }
        }

        if (visible_contrib_sum > 1e-9) {
            // Tier 1: sector-weighted (normal path)
            for (int j = 0; j < N; ++j) {
                if (visible[j] && raw_weights[j] > 1e-9 && edge_factor[j] > 0.0) {
                    bv.weight[j] = static_cast<float>(
                        raw_weights[j] * edge_factor[j] / visible_contrib_sum);
                }
            }
        } else if (edge_only_sum > 1e-9) {
            // Tier 2: geometry-weighted (D3 re-fallback — sector model
            // assigned all weight to an invisible camera, so distribute
            // over all geometrically-visible cameras by edge proximity).
            for (int j = 0; j < N; ++j) {
                if (visible[j] && edge_factor[j] > 0.0) {
                    bv.weight[j] = static_cast<float>(
                        edge_factor[j] / edge_only_sum);
                }
            }
        }
        // Tier 3: no visible camera with non-zero feather → all weights stay 0.
    }
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
