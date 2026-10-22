#pragma once

#include "camera_model/camera_model.hpp"

namespace avm_stitch {

class LutCameraModel final : public CameraModel {
  public:
    LutCameraModel(std::vector<std::array<double, 2>> points,
                   double pixel_size_mm, int width, int height,
                   double cx, double cy);

    void ProjectPoints(std::span<const cv::Vec3d> cam_points,
                       std::span<cv::Vec2d> out_uv,
                       std::span<uint8_t> out_valid) const override;

  private:
    double RadiusPxForAngle(double theta_deg) const;

    std::vector<std::array<double, 2>> points_;
    double pixel_size_mm_;
};

}
