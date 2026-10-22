#pragma once

// formula; the batch loop delegates each camera-frame point to it.

#include "camera_model/camera_model.hpp"

namespace avm_stitch {

class KbCameraModel final : public CameraModel {
public:
    // Flat KbParams (width/height/fx/fy/cx/cy/k1..k4/fisheye_radius).
    explicit KbCameraModel(const KbParams& p);

            void ProjectPoints(std::span<const cv::Vec3d> cam_points,
                       std::span<cv::Vec2d> out_uv,
                       std::span<uint8_t> out_valid) const override;

    // θ-polynomial projection (camera-frame point -> pixel). r_xy < kEps
    // returns (cx, cy).
    cv::Vec2d ProjectCameraPoint(const cv::Vec3d& P_cam) const override;

private:
    double fx_, fy_;
    double k1_, k2_, k3_, k4_;
};

}
