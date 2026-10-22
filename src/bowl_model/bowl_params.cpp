#include "bowl_model/bowl_params.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace avm_stitch {

BowlParams::BowlParams(double car_length, double car_width, double round_l,
                       double step, double r_outter, double k_slop,
                       double z_exp, double max_z, cv::Vec2d car_center,
                       BowlType type)
    : car_length_(car_length), car_width_(car_width), round_l_(round_l),
      step_(step), r_outter_(r_outter), k_slop_(k_slop), z_exp_(z_exp),
      max_z_(max_z), type_(type), car_center_(car_center) {}

double BowlParams::rd(double x, double y) const {
            double car_l_half = car_length_ / 2.0;
    double car_w_half = car_width_ / 2.0;
    double rel_x = x - car_center_[0];
    double rel_y = y - car_center_[1];

    double dx = std::max(0.0, std::abs(rel_x) - (car_l_half + round_l_));
    double dy = std::max(0.0, std::abs(rel_y) - (car_w_half + round_l_));
    return std::sqrt(dx * dx + dy * dy);
}

double BowlParams::bowl_z(double rd) const {
    if (rd <= 0.0) return 0.0;
    return k_slop_ * std::pow(rd, z_exp_);
}

int BowlParams::grid_segments() const {
    return static_cast<int>(std::lround(r_outter_ / step_) * 2);
}

namespace {

void ComputeBowlNormals(const std::vector<int>& grid_to_index,
                        int rows, int cols,
                        const std::vector<cv::Vec3d>& vertices,
                        std::vector<cv::Vec3d>& normals) {
    for (int i = 0; i <= rows; ++i) {
        for (int j = 0; j <= cols; ++j) {
            int idx = grid_to_index[i * (cols + 1) + j];
            if (idx < 0) continue;
            double z = vertices[idx][2];
            if (z < 1e-9) {
                normals[idx] = cv::Vec3d(0.0, 0.0, 1.0);
                continue;
            }
            double dz_dx = 0.0, dz_dy = 0.0;
            int count_x = 0, count_y = 0;
                                    auto add_neighbor = [&](int ni, int nj, int side, double& acc, int& count) {
                if (ni < 0 || ni > rows || nj < 0 || nj > cols) return;
                int nidx = grid_to_index[ni * (cols + 1) + nj];
                if (nidx < 0) return;
                acc += side * (vertices[nidx][2] - z);
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
            normals[idx] = (len > 1e-9) ? n / len : cv::Vec3d(0.0, 0.0, 1.0);
        }
    }
}

}
void BowlParams::GenerateBlockBoatMesh(std::vector<cv::Vec3d>& vertices,
                                       std::vector<cv::Vec3d>& normals,
                                       std::vector<std::array<int, 3>>& triangles) const {
    const int rows = grid_segments(), cols = grid_segments();

    std::vector<int> grid_to_index((rows + 1) * (cols + 1), -1);
    int vertex_count = 0;

    for (int i = 0; i <= rows; ++i) {
        for (int j = 0; j <= cols; ++j) {
            double u = static_cast<double>(i) / rows * 2.0 - 1.0;
            double v = static_cast<double>(j) / cols * 2.0 - 1.0;
                                    double x = car_center()[0] + u * r_outter();
            double y = car_center()[1] + v * r_outter();

            const double rd = this->rd(x, y);
            if (rd > r_outter() + 1e-9) continue;

            double z = bowl_z(rd);
            if (z > max_z()) continue;

            vertices.push_back(cv::Vec3d(x, y, z));
            normals.push_back(cv::Vec3d(0.0, 0.0, 1.0));
            grid_to_index[i * (cols + 1) + j] = vertex_count++;
        }
    }

    ComputeBowlNormals(grid_to_index, rows, cols, vertices, normals);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            int p1 = grid_to_index[i * (cols + 1) + j];
            int p2 = grid_to_index[i * (cols + 1) + (j + 1)];
            int p3 = grid_to_index[(i + 1) * (cols + 1) + j];
            int p4 = grid_to_index[(i + 1) * (cols + 1) + (j + 1)];
            if (p1 >= 0 && p2 >= 0 && p3 >= 0 && p4 >= 0) {
                triangles.push_back({p1, p3, p2});
                triangles.push_back({p2, p3, p4});
            }
        }
    }
}

BowlParams DeriveBowlParams(double car_length, double car_width, double axis_x) {
                constexpr double kRefLength = 5.6;
    const double scale = car_length / kRefLength;

                            return BowlParams(car_length,
                      car_width,
                      2.0 * scale,                                            0.2 * scale,                                            kCalibratedAnchorRoutter * scale,                       0.09 * std::pow(scale, -1.2),                           2.2,                                                    8.6 * scale,                                            cv::Vec2d(axis_x / 2.0, 0.0));    }

}
