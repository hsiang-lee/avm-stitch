#pragma once

#include "camera_model/camera_model.hpp"

namespace avm_stitch {

class OcamCameraModel final : public CameraModel {
  public:
    OcamCameraModel(std::vector<double> pol, std::vector<double> invpol,
                    double cx, double cy, double c, double d, double e,
                    int width, int height);

    void ProjectPoints(std::span<const cv::Vec3d> cam_points,
                       std::span<cv::Vec2d> out_uv,
                       std::span<uint8_t> out_valid) const override;

  private:
    std::vector<double> pol_;
    std::vector<double> invpol_;
    double c_, d_, e_;
};

}
