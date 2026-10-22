#include "camera_model/kb_camera_model.hpp"

#include <cmath>
#include <span>
#include <stdexcept>

namespace avm_stitch {

KbCameraModel::KbCameraModel(const KbParams& p)
    : CameraModel(p.width, p.height, p.cx, p.cy, p.fisheye_radius),
      fx_(p.fx), fy_(p.fy),
      k1_(p.k1), k2_(p.k2), k3_(p.k3), k4_(p.k4) {}

// verbatim from the retired hand-written ProjectCameraPoint). The batch
// loop below shares this exact code path; the extrinsics transform lives in
// CameraExtrinsics and is applied by the caller (CameraModel consumes
// camera-frame points by contract).
cv::Vec2d KbCameraModel::ProjectCameraPoint(const cv::Vec3d& P_cam) const {
    const double x = P_cam[0], y = P_cam[1], z = P_cam[2];
    const double r_xy = std::sqrt(x * x + y * y);

    if (r_xy < kEps) {
        return cv::Vec2d(cx_, cy_);
    }

    double theta = std::atan2(r_xy, z); // ∈ [0, π]
    double theta2 = theta * theta;
    double theta_d = theta * (1.0 + k1_ * theta2 + k2_ * theta2 * theta2 + k3_ * theta2 * theta2 * theta2 + k4_ * theta2 * theta2 * theta2 * theta2);
    double factor = theta_d / r_xy;
    return cv::Vec2d(cx_ + fx_ * factor * x,
                     cy_ + fy_ * factor * y);
}

void KbCameraModel::ProjectPoints(std::span<const cv::Vec3d> cam_points,
                                  std::span<cv::Vec2d> out_uv,
                                  std::span<uint8_t> out_valid) const {
    RequireEqualLengths(cam_points, out_uv, out_valid);

    for (size_t i = 0; i < cam_points.size(); ++i) {
        out_uv[i] = ProjectCameraPoint(cam_points[i]);
        out_valid[i] = RectContains(out_uv[i][0], out_uv[i][1]);
    }
}

}
