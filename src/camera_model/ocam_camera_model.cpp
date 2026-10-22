#include "camera_model/ocam_camera_model.hpp"

#include <cmath>
#include <utility>

namespace avm_stitch {

OcamCameraModel::OcamCameraModel(std::vector<double> pol,
                                 std::vector<double> invpol,
                                 double cx, double cy, double c, double d,
                                 double e, int width, int height)
    : CameraModel(width, height, cx, cy),
      pol_(std::move(pol)),
      invpol_(std::move(invpol)),
      c_(c), d_(d), e_(e) {}

void OcamCameraModel::ProjectPoints(std::span<const cv::Vec3d> cam_points,
                                    std::span<cv::Vec2d> out_uv,
                                    std::span<uint8_t> out_valid) const {
    RequireEqualLengths(cam_points, out_uv, out_valid);

    for (size_t i = 0; i < cam_points.size(); ++i) {
        const double x = cam_points[i][0];
        const double y = cam_points[i][1];
        const double z = cam_points[i][2];

        const double r_xy = std::sqrt(x * x + y * y);
        if (r_xy < 1e-12) {
            out_uv[i] = cv::Vec2d(cx_, cy_);
            out_valid[i] = (z > 0.0) ? RectContains(cx_, cy_) : 0;
            continue;
        }
        if (z <= 0.0) {
            out_uv[i] = cv::Vec2d(cx_, cy_);
            out_valid[i] = 0;
            continue;
        }

        const double theta = std::atan(-z / r_xy);
        double rho = invpol_.empty() ? 0.0 : invpol_[0];
        double t_i = 1.0;
        for (size_t k = 1; k < invpol_.size(); ++k) {
            t_i *= theta;
            rho += t_i * invpol_[k];
        }
        const double scale = rho / r_xy;
        const double px = x * scale;
        const double py = y * scale;
        const double u = px * c_ + py * d_ + cx_;
        const double v = px * e_ + py + cy_;

        out_uv[i] = cv::Vec2d(u, v);
        out_valid[i] = RectContains(u, v);
    }
}

}
