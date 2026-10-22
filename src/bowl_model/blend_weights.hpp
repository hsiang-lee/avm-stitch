#pragma once


#include "camera_model/camera_extrinsics.hpp"

#include <opencv2/core.hpp>
#include <vector>

namespace avm_stitch {

double ComputeBlendHalfAngle(int num_cameras);

std::vector<double> ComputeBlendWeights(
    const cv::Vec3d& P_world,
    const std::vector<CameraExtrinsics>& extrinsics,
    const cv::Vec2d& pole = cv::Vec2d(0.0, 0.0));

}
