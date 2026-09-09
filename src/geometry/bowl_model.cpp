#include "geometry/bowl_model.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>

namespace avm_stitch {

// Main constructor: three-step mesh generation (verbatim from the former
// header-only implementation; only the `inline` keyword was dropped).
BowlModel::BowlModel(const BowlParams& params) {
    const double x_half = params.car_length / 2.0 + params.round_l;
    const double y_half = params.car_width / 2.0 + params.round_l;

    // Round (not truncate) so e.g. 12.6/0.1 = 126, matching the spec grid size.
    rows_ = cols_ = std::lround(params.r_outter / params.step) * 2;

    // grid_to_index: -1 means clipped
    std::vector<int> grid_to_index((rows_ + 1) * (cols_ + 1), -1);
    int vertex_count = 0;

    // Step 1: generate vertices (VCS coordinates)
    for (int i = 0; i <= rows_; ++i) {
        for (int j = 0; j <= cols_; ++j) {
            double u = static_cast<double>(i) / rows_ * 2.0 - 1.0;
            double v = static_cast<double>(j) / cols_ * 2.0 - 1.0;
            double x = u * params.r_outter;
            double y = v * params.r_outter;

            double dx = std::max(0.0, std::abs(x) - x_half);
            double dy = std::max(0.0, std::abs(y) - y_half);
            double rd = std::sqrt(dx * dx + dy * dy);

            if (rd > params.r_outter + 1e-9) continue;

            double z = 0.0;
            if (rd > 0.0) {
                z = params.k_slop * std::pow(rd, params.z_exp);
            }
            if (z > params.max_z) continue;

            vertices_.push_back(cv::Vec3d(x, y, z));
            normals_.push_back(cv::Vec3d(0.0, 0.0, 1.0));   // placeholder
            grid_to_index[i * (cols_ + 1) + j] = vertex_count++;
        }
    }

    // Step 2: recompute normals for curved vertices using finite difference
    ComputeBowlNormals(*this, grid_to_index, params, rows_, cols_);

    // Step 3: build triangle indices (only quads with all 4 corners retained)
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

BowlModel::BowlModel(std::vector<cv::Vec3d> vertices,
                     std::vector<cv::Vec3d> normals)
    : vertices_(std::move(vertices)), normals_(std::move(normals)) {}

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
            if (count_x > 0) dz_dx /= (count_x * params.step);
            if (count_y > 0) dz_dy /= (count_y * params.step);

            cv::Vec3d n(-dz_dx, -dz_dy, 1.0);
            double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
            mesh.normals_[idx] = (len > 1e-9) ? n / len : cv::Vec3d(0.0, 0.0, 1.0);
        }
    }
}

} // namespace avm_stitch
