#pragma once

// N-camera blend weights by azimuth sector (pure math).
// Interface only — implementation in blend_weights.cpp.

#include "geometry/projection.hpp"

#include <opencv2/core.hpp>
#include <vector>

namespace avm_worker { namespace avm_stitch {

/// Compute sector boundary angles (degrees) for N cameras sorted by azimuth.
/// @return N boundaries in [0, 360), midpoints between adjacent sorted cameras.
std::vector<double> ComputeSectorBoundaries(
    const std::vector<CameraExtrinsics>& extrinsics);

/// Compute blend half-angle (degrees) for N cameras.
/// Formula: 13 deg * (360/N) / 90 = 52/N degrees.
double ComputeBlendHalfAngle(int num_cameras);

/// Compute blend weights for a world point across N cameras.
/// @param P_world   world point (VCS, metres)
/// @param extrinsics N camera extrinsics (T_wc)
/// @return weights[i] in [0,1], sum = 1.0
std::vector<double> ComputeBlendWeights(
    const cv::Vec3d& P_world,
    const std::vector<CameraExtrinsics>& extrinsics);

}} // namespace avm_worker::avm_stitch
