#include "bowl_model/bowl_params.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace avm_stitch {

BowlParams::BowlParams(double car_length, double car_width, double flat_platform_radius,
                       double grid_step, double outer_radius,
                       double slope_coefficient, double height_exponent,
                       double max_height, cv::Vec2d car_center)
    : car_length_(car_length), car_width_(car_width),
      flat_platform_radius_(flat_platform_radius), grid_step_(grid_step),
      outer_radius_(outer_radius), slope_coefficient_(slope_coefficient),
      height_exponent_(height_exponent), max_height_(max_height),
      car_center_(car_center) {}

double BowlParams::DistanceFromFlatPlatform(double x, double y) const {
    double half_car_length = car_length_ / 2.0;
    double half_car_width = car_width_ / 2.0;
    double offset_x = x - car_center_[0];
    double offset_y = y - car_center_[1];

    double overshoot_x =
        std::max(0.0, std::abs(offset_x) - (half_car_length + flat_platform_radius_));
    double overshoot_y =
        std::max(0.0, std::abs(offset_y) - (half_car_width + flat_platform_radius_));
    return std::sqrt(overshoot_x * overshoot_x + overshoot_y * overshoot_y);
}

double BowlParams::BowlHeight(double distance_from_flat_platform) const {
    if (distance_from_flat_platform <= 0.0)
        return 0.0;
    return slope_coefficient_ *
           std::pow(distance_from_flat_platform, height_exponent_);
}

int BowlParams::GridSegmentsPerAxis() const {
    return static_cast<int>(std::lround(outer_radius_ / grid_step_) * 2);
}

namespace {

void ComputeBowlNormals(const std::vector<int>& grid_to_index,
                        int rows, int cols,
                        const std::vector<cv::Vec3d>& vertices,
                        std::vector<cv::Vec3d>& normals) {
    for (int i = 0; i <= rows; ++i) {
        for (int j = 0; j <= cols; ++j) {
            int idx = grid_to_index[i * (cols + 1) + j];
            if (idx < 0)
                continue;
            double z = vertices[idx][2];
            if (z < 1e-9) {
                normals[idx] = cv::Vec3d(0.0, 0.0, 1.0);
                continue;
            }
            double dz_dx = 0.0, dz_dy = 0.0;
            int count_x = 0, count_y = 0;
            auto add_neighbor = [&](int ni, int nj, int side,
                                    double& acc, int& count) {
                if (ni < 0 || ni > rows || nj < 0 || nj > cols)
                    return;
                int nidx = grid_to_index[ni * (cols + 1) + nj];
                if (nidx < 0)
                    return;
                acc += side * (vertices[nidx][2] - z);
                ++count;
            };
            add_neighbor(i, j - 1, -1, dz_dx, count_x);
            add_neighbor(i, j + 1, +1, dz_dx, count_x);
            add_neighbor(i - 1, j, -1, dz_dy, count_y);
            add_neighbor(i + 1, j, +1, dz_dy, count_y);
            if (count_x > 0)
                dz_dx /= count_x;
            if (count_y > 0)
                dz_dy /= count_y;

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
    const int rows = GridSegmentsPerAxis(), cols = GridSegmentsPerAxis();

    std::vector<int> grid_to_index((rows + 1) * (cols + 1), -1);
    int vertex_count = 0;

    for (int i = 0; i <= rows; ++i) {
        for (int j = 0; j <= cols; ++j) {
            double u = static_cast<double>(i) / rows * 2.0 - 1.0;
            double v = static_cast<double>(j) / cols * 2.0 - 1.0;
            double x = CarCenter()[0] + u * OuterRadius();
            double y = CarCenter()[1] + v * OuterRadius();

            const double distance_from_platform = DistanceFromFlatPlatform(x, y);
            if (distance_from_platform > OuterRadius() + 1e-9)
                continue;

            double z = BowlHeight(distance_from_platform);
            if (z > MaxHeight())
                continue;

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

std::vector<ViewPose> BowlParams::DefaultViewPoses() const {
    const double scale = outer_radius_ / kCalibratedAnchorOuterRadius;
    const double cx = car_center_[0];
    const double cy = car_center_[1];
    const double z_ground = 0.2 * scale;
    return {
        {"3d_front", {cx + 5.8 * scale, cy, 2.0 * scale}, {cx, cy, z_ground}, {0, 0, 1}, 60.0},
        {"3d_rear", {cx - 5.8 * scale, cy, 2.0 * scale}, {cx, cy, z_ground}, {0, 0, 1}, 60.0},
        {"3d_left", {cx, cy + 5.8 * scale, 2.0 * scale}, {cx, cy, z_ground}, {0, 0, 1}, 60.0},
        {"3d_right", {cx, cy - 5.8 * scale, 2.0 * scale}, {cx, cy, z_ground}, {0, 0, 1}, 60.0},
    };
}

ClipPlanes BowlParams::ClipPlanesFor(double max_eye_to_target) const {
    constexpr double kFarMargin = 5.3553572528096565;
    constexpr double kNearRel = 0.01;
    const double far = kFarMargin * (max_eye_to_target + outer_radius_);
    const double near = std::clamp(outer_radius_ * kNearRel, 1.0e-3, far / 1000.0);
    return {near, far};
}

ViewPose BowlParams::HomeViewPose() const {
    const double cx = car_center_[0];
    const double cy = car_center_[1];
    const double scale = outer_radius_ / kCalibratedAnchorOuterRadius;
    return {"", {10.0 * scale + cx, -6.0 * scale, 6.0 * scale - cy}, {cx, 0.0, 0.5 * scale - cy}, {0.0, 0.0, 1.0}, 60.0};
}

double BowlParams::HomeViewDistance() const {
    return 12.0 * outer_radius_ / kCalibratedAnchorOuterRadius;
}

BowlParams BowlParams::ForCarSize(double car_length, double car_width, double axis_x) {
    constexpr double kReferenceCarLength = 5.6;
    const double scale = car_length / kReferenceCarLength;

    return BowlParams(car_length, car_width, 2.0 * scale, 0.2 * scale,
                      kCalibratedAnchorOuterRadius * scale,
                      0.09 * std::pow(scale, -1.2), 2.2, 8.6 * scale,
                      cv::Vec2d(axis_x / 2.0, 0.0));
}

}
