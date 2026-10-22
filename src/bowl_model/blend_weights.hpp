#pragma once


#include "camera_model/camera_extrinsics.hpp"
#include "common/angle_utils.hpp"

#include <opencv2/core.hpp>
#include <utility>
#include <vector>

namespace avm_stitch {

class BlendWeightsCalculator {
  public:
    explicit BlendWeightsCalculator(
        const std::vector<CameraExtrinsics>& extrinsics,
        cv::Vec2d pole = cv::Vec2d(0.0, 0.0));
    std::vector<double> WeightsFor(const cv::Vec3d& P_world) const;
    static double ComputeBlendHalfAngle(int num_cameras);

  private:
    std::vector<std::pair<double, int>> sorted_;
    cv::Vec2d pole_;
    int num_cameras_ = 0;
};

}
