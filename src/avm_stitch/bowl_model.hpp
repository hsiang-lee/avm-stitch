#pragma once

// Bowl model: car-parameterized bowl mesh generation (pure math, header-only).
// Algorithm mirrors the reference `create_block_boat_model` (bowlModel.hpp):
//   z = k_slop * rd^z_exp outside the rounded-rectangle flat region,
//   finite-difference normals on the curved rim.

#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace avm_worker { namespace avm_stitch {

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

    // Path priority: task["calibration"]["bowl"] -> task["bowl"] -> defaults.
    // Missing fields keep defaults; numeric fields accept int or double.
    // Malformed (non-numeric) fields fall back to defaults and never throw.
    static BowlParams FromJson(const nlohmann::json& task) {
        BowlParams p;
        auto read = [&](const nlohmann::json& obj) {
            // Safe extraction: missing or non-numeric (string/null/bool/etc.)
            // values fall back to the default — do not let get<double>() throw.
            auto read_param = [&obj](const char* key, double& target) {
                auto it = obj.find(key);
                if (it != obj.end() && it->is_number()) target = it->get<double>();
            };
            read_param("car_length", p.car_length);
            read_param("car_width", p.car_width);
            read_param("round_l", p.round_l);
            read_param("step", p.step);
            read_param("r_outter", p.r_outter);
            read_param("k_slop", p.k_slop);
            read_param("z_exp", p.z_exp);
            read_param("max_z", p.max_z);
        };
        if (task.contains("calibration") && task["calibration"].contains("bowl")) {
            read(task["calibration"]["bowl"]);
        } else if (task.contains("bowl")) {
            read(task["bowl"]);
        }
        return p;
    }
};

struct BowlMesh {
    std::vector<cv::Vec3d> vertices;
    std::vector<cv::Vec3d> normals;
    std::vector<std::array<int, 3>> triangles;
    int rows = 0;
    int cols = 0;
};

namespace detail {

// Finite-difference normals: flat (z ~ 0) -> (0,0,1); curved -> unit normal
// derived from dz/dx, dz/dy estimated from retained grid neighbours.
inline void ComputeBowlNormals(BowlMesh& mesh, const std::vector<int>& grid_to_index,
                               const BowlParams& params, int rows, int cols) {
    for (int i = 0; i <= rows; ++i) {
        for (int j = 0; j <= cols; ++j) {
            int idx = grid_to_index[i * (cols + 1) + j];
            if (idx < 0) continue;
            double z = mesh.vertices[idx][2];
            if (z < 1e-9) {
                mesh.normals[idx] = cv::Vec3d(0.0, 0.0, 1.0);
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
                acc += side * (mesh.vertices[nidx][2] - z);
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
            mesh.normals[idx] = (len > 1e-9) ? n / len : cv::Vec3d(0.0, 0.0, 1.0);
        }
    }
}

} // namespace detail

inline BowlMesh CreateBowlMesh(const BowlParams& params) {
    BowlMesh mesh;

    const double x_half = params.car_length / 2.0 + params.round_l;
    const double y_half = params.car_width / 2.0 + params.round_l;

    // Round (not truncate) so e.g. 12.6/0.1 = 126, matching the spec grid size.
    const int rows = std::lround(params.r_outter / params.step) * 2;
    const int cols = rows;
    mesh.rows = rows;
    mesh.cols = cols;

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

            mesh.vertices.push_back(cv::Vec3d(x, y, z));
            mesh.normals.push_back(cv::Vec3d(0.0, 0.0, 1.0));   // placeholder
            grid_to_index[i * (cols + 1) + j] = vertex_count++;
        }
    }

    // Step 2: recompute normals for curved vertices using finite difference
    detail::ComputeBowlNormals(mesh, grid_to_index, params, rows, cols);

    // Step 3: build triangle indices (only quads with all 4 corners retained)
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            int p1 = grid_to_index[i * (cols + 1) + j];
            int p2 = grid_to_index[i * (cols + 1) + (j + 1)];
            int p3 = grid_to_index[(i + 1) * (cols + 1) + j];
            int p4 = grid_to_index[(i + 1) * (cols + 1) + (j + 1)];
            if (p1 >= 0 && p2 >= 0 && p3 >= 0 && p4 >= 0) {
                mesh.triangles.push_back({p1, p3, p2});
                mesh.triangles.push_back({p2, p3, p4});
            }
        }
    }

    return mesh;
}

}} // namespace avm_worker::avm_stitch
