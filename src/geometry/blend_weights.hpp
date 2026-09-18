#pragma once

// N-camera blend weights by azimuth sector (pure math).
// Interface only — implementation in blend_weights.cpp.

#include "geometry/projection.hpp"

#include <opencv2/core.hpp>
#include <vector>

namespace avm_stitch {

/// Compute blend half-angle (degrees) for N cameras.
/// Formula: 13 deg * (360/N) / 90 = 52/N degrees.
double ComputeBlendHalfAngle(int num_cameras);

/// Compute blend weights for a world point across N cameras.
/// @param P_world   world point (VCS, metres)
/// @param extrinsics N camera extrinsics (T_wc)
/// @param pole     sector pole (car_center), default (0,0) for backward compat.
///                 Point azimuth computed relative to pole: atan2(P_y-pole_y, P_x-pole_x).
/// @return weights[i] in [0,1], sum = 1.0
std::vector<double> ComputeBlendWeights(
    const cv::Vec3d& P_world,
    const std::vector<CameraExtrinsics>& extrinsics,
    const cv::Vec2d& pole = cv::Vec2d(0.0, 0.0));

} // namespace avm_stitch
