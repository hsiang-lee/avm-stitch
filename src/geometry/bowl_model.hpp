#pragma once

// Bowl model: car-parameterized bowl mesh generation (pure math, header-only).
// Algorithm mirrors the reference `create_block_boat_model` (bowlModel.hpp):
//   z = k_slop * rd^z_exp outside the rounded-rectangle flat region,
//   finite-difference normals on the curved rim.

#include <opencv2/core.hpp>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <vector>

namespace avm_stitch {

struct BowlParams {
    double car_length = 5.6;
    double car_width = 2.2;
    double round_l = 2.0;
    double step = 0.1;
    double r_outter = 12.6;
    double k_slop = 0.09;
    double z_exp = 2.2;
    double max_z = 8.6;

    static BowlParams Default() { return BowlParams{}; }
};

// Bowl surface triangle mesh generated from car parameters. Immutable data
// holder: read via the fine-grained index accessors (the internal container
// layout is an implementation detail, free to change e.g. to SoA).
class BowlMesh {
public:
    /// Generate the bowl mesh (VCS coordinates) from car parameters.
    explicit BowlMesh(const BowlParams& params);

    /// Wrap pre-built vertex data (for tests / custom geometry): normals are
    /// taken as-is, no grid/normal recomputation, no triangles.
    BowlMesh(std::vector<cv::Vec3d> vertices, std::vector<cv::Vec3d> normals);

    // --- fine-grained read access (index + count, no container exposure) ---
    /// Number of vertices (always == normal_count(), a construction invariant).
    int vertex_count() const { return static_cast<int>(vertices_.size()); }
    /// Vertex i in VCS coordinates. Precondition: 0 <= i < vertex_count().
    const cv::Vec3d& vertex(int i) const {
        assert(i >= 0 && i < vertex_count());
        return vertices_[static_cast<size_t>(i)];
    }
    /// Number of normals (always == vertex_count(), a construction invariant).
    int normal_count() const { return static_cast<int>(normals_.size()); }
    /// Unit normal i. Precondition: 0 <= i < normal_count().
    const cv::Vec3d& normal(int i) const {
        assert(i >= 0 && i < normal_count());
        return normals_[static_cast<size_t>(i)];
    }
    /// Number of triangles.
    int triangle_count() const { return static_cast<int>(triangles_.size()); }
    /// Triangle i as vertex indices (by value: small object).
    /// Precondition: 0 <= i < triangle_count().
    std::array<int, 3> triangle(int i) const {
        assert(i >= 0 && i < triangle_count());
        return triangles_[static_cast<size_t>(i)];
    }

    int rows() const { return rows_; }
    int cols() const { return cols_; }

private:
    // Finite-difference normals: flat (z ~ 0) -> (0,0,1); curved -> unit normal
    // derived from dz/dx, dz/dy estimated from retained grid neighbours.
    static void ComputeBowlNormals(BowlMesh& mesh,
                                   const std::vector<int>& grid_to_index,
                                   const BowlParams& params, int rows, int cols);

    std::vector<cv::Vec3d> vertices_;
    std::vector<cv::Vec3d> normals_;
    std::vector<std::array<int, 3>> triangles_;
    int rows_ = 0;
    int cols_ = 0;
};

inline BowlMesh::BowlMesh(const BowlParams& params) {
    const double x_half = params.car_length / 2.0 + params.round_l;
    const double y_half = params.car_width / 2.0 + params.round_l;

    // Round (not truncate) so e.g. 12.6/0.1 = 126, matching the spec grid size.
    const int rows = std::lround(params.r_outter / params.step) * 2;
    const int cols = rows;
    rows_ = rows;
    cols_ = cols;

    // grid_to_index: -1 means clipped
    std::vector<int> grid_to_index((rows + 1) * (cols + 1), -1);
    int vertex_count = 0;

    // Step 1: generate vertices (VCS coordinates)
    for (int i = 0; i <= rows; ++i) {
        for (int j = 0; j <= cols; ++j) {
            double u = static_cast<double>(i) / rows * 2.0 - 1.0;
            double v = static_cast<double>(j) / cols * 2.0 - 1.0;
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
            grid_to_index[i * (cols + 1) + j] = vertex_count++;
        }
    }

    // Step 2: recompute normals for curved vertices using finite difference
    ComputeBowlNormals(*this, grid_to_index, params, rows, cols);

    // Step 3: build triangle indices (only quads with all 4 corners retained)
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            int p1 = grid_to_index[i * (cols + 1) + j];
            int p2 = grid_to_index[i * (cols + 1) + (j + 1)];
            int p3 = grid_to_index[(i + 1) * (cols + 1) + j];
            int p4 = grid_to_index[(i + 1) * (cols + 1) + (j + 1)];
            if (p1 >= 0 && p2 >= 0 && p3 >= 0 && p4 >= 0) {
                triangles_.push_back({p1, p3, p2});
                triangles_.push_back({p2, p3, p4});
            }
        }
    }
}

inline BowlMesh::BowlMesh(std::vector<cv::Vec3d> vertices,
                          std::vector<cv::Vec3d> normals)
    : vertices_(std::move(vertices)), normals_(std::move(normals)) {}

inline void BowlMesh::ComputeBowlNormals(BowlMesh& mesh,
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
