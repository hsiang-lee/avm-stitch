#include "camera_model/lut_camera_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace avm_stitch {

LutCameraModel::LutCameraModel(std::vector<std::array<double, 2>> points,
                               double pixel_size_mm, int width, int height,
                               double cx, double cy)
    : CameraModel(width, height, cx, cy),
      points_(std::move(points)),
      pixel_size_mm_(pixel_size_mm) {}

void LutCameraModel::ProjectPoints(std::span<const cv::Vec3d> cam_points,
                                   std::span<cv::Vec2d> out_uv,
                                   std::span<uint8_t> out_valid) const {
    RequireEqualLengths(cam_points, out_uv, out_valid);

    for (size_t i = 0; i < cam_points.size(); ++i) {
        const double x = cam_points[i][0];
        const double y = cam_points[i][1];
        const double z = cam_points[i][2];
        const double r_xy = std::sqrt(x * x + y * y);

        if (r_xy < kEps) {
            out_uv[i] = cv::Vec2d(cx_, cy_);
            out_valid[i] = RectContains(cx_, cy_);
            continue;
        }

        const double theta_deg = std::atan2(r_xy, z) * 180.0 / M_PI;
        const double r_px = RadiusPxForAngle(theta_deg);
        const double scale = r_px / r_xy;
        const double u = cx_ + scale * x;
        const double v = cy_ + scale * y;

        out_uv[i] = cv::Vec2d(u, v);
        out_valid[i] = RectContains(u, v);
    }
}

double LutCameraModel::RadiusPxForAngle(double theta_deg) const {
    const auto& xs = points_;
    auto at = [&](size_t i) { return xs[i][1]; };

    if (theta_deg <= xs.front()[0]) {
        return at(0) / pixel_size_mm_;
    }
    if (theta_deg >= xs.back()[0]) {
        return at(xs.size() - 1) / pixel_size_mm_;
    }
    const auto it = std::upper_bound(
        xs.begin(), xs.end(), theta_deg,
        [](double value, const std::array<double, 2>& point) {
            return value < point[0];
        });
    const size_t hi = static_cast<size_t>(it - xs.begin());
    const size_t lo = hi - 1;
    const double t = (theta_deg - xs[lo][0]) / (xs[hi][0] - xs[lo][0]);
    return (at(lo) + t * (at(hi) - at(lo))) / pixel_size_mm_;
}

}
