#include "geometry/kb_camera_model.hpp"

#include "geometry/projection.hpp"

#include <cmath>
#include <span>
#include <stdexcept>
#include <utility>

namespace avm_stitch {

KbCameraModel::KbCameraModel(const CameraIntrinsics& intr)
    : CameraModel(intr.width, intr.height, intr.cx, intr.cy), intr_(intr) {}

// Equidistant θ-polynomial projection, numerically equivalent to
// ProjectWorldToFishEye (projection.hpp) for camera-frame points — the
// formula below is the same code with the extrinsics transform factored out
// (CameraModel consumes camera-frame points by contract).
void KbCameraModel::ProjectPoints(std::span<const cv::Vec3d> cam_points,
                                  std::span<cv::Vec2d> out_uv,
                                  std::span<uint8_t> out_valid) const {
    RequireEqualLengths(cam_points, out_uv, out_valid);

    for (size_t i = 0; i < cam_points.size(); ++i) {
        const double x = cam_points[i][0];
        const double y = cam_points[i][1];
        const double z = cam_points[i][2];
        const double r_xy = std::sqrt(x * x + y * y);

        if (r_xy < kEps) {
            out_uv[i] = cv::Vec2d(intr_.cx, intr_.cy);
            out_valid[i] = RectContains(intr_.cx, intr_.cy);
            continue;
        }

        const double theta = std::atan2(r_xy, z);          // ∈ [0, π]
        const double theta2 = theta * theta;
        const double theta_d = theta * (1.0 + intr_.k1 * theta2
                                              + intr_.k2 * theta2 * theta2
                                              + intr_.k3 * theta2 * theta2 * theta2
                                              + intr_.k4 * theta2 * theta2 * theta2 * theta2);
        const double factor = theta_d / r_xy;
        const double u = intr_.cx + intr_.fx * factor * x;
        const double v = intr_.cy + intr_.fy * factor * y;

        out_uv[i] = cv::Vec2d(u, v);
        out_valid[i] = RectContains(u, v);
    }
}

} // namespace avm_stitch
