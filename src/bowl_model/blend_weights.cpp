#include "bowl_model/blend_weights.hpp"

#include "common/angle_utils.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace avm_stitch {

namespace {

double ShortestArcDeg(double a, double b) {
    double dist = std::abs(NormalizeAngleDeg(a - b));
    if (dist > 180.0)
        dist = 360.0 - dist;
    return dist;
}

double SectorBoundaryDeg(double az1, double az2) {
    if (az2 < az1)
        az2 += 360.0;
    return NormalizeAngleDeg((az1 + az2) / 2.0);
}

} // namespace

BlendWeightsCalculator::BlendWeightsCalculator(
    const std::vector<CameraExtrinsics>& extrinsics, cv::Vec2d pole)
    : pole_(pole) {
    num_cameras_ = static_cast<int>(extrinsics.size());
    sorted_.reserve(num_cameras_);
    for (int i = 0; i < num_cameras_; ++i)
        sorted_.emplace_back(extrinsics[i].AzimuthDeg(), i);
    std::sort(sorted_.begin(), sorted_.end());
}

double BlendWeightsCalculator::ComputeBlendHalfAngle(int num_cameras) {
    if (num_cameras <= 0)
        return 0.0;
    return 52.0 / num_cameras;
}

int BlendWeightsCalculator::NearestCameraIndex(double theta_deg) const {
    int best = 0;
    double best_dist = 360.0;
    for (int j = 0; j < num_cameras_; ++j) {
        const double d = ShortestArcDeg(theta_deg, sorted_[j].first);
        if (d < best_dist) {
            best_dist = d;
            best = j;
        }
    }
    return best;
}

bool BlendWeightsCalculator::TryBlend(
    int near_index, double theta_deg, double blend_half_deg,
    int best_index, std::vector<double>& weights) const {
    const int b = (near_index + 1) % num_cameras_;
    const double boundary =
        SectorBoundaryDeg(sorted_[near_index].first, sorted_[b].first);
    const double dist = ShortestArcDeg(theta_deg, boundary);
    if (dist > blend_half_deg)
        return false;
    const double t = 0.5 + dist / (2.0 * blend_half_deg);
    const double w_near = 0.5 * (1.0 - std::cos(t * kPi));
    const double w_far = 1.0 - w_near;
    if (best_index == near_index) {
        weights[sorted_[near_index].second] = w_near;
        weights[sorted_[b].second] = w_far;
    } else {
        weights[sorted_[b].second] = w_near;
        weights[sorted_[near_index].second] = w_far;
    }
    return true;
}

std::vector<double>
BlendWeightsCalculator::WeightsFor(const cv::Vec3d& P_world) const {
    if (num_cameras_ == 0)
        return {};
    std::vector<double> weights(num_cameras_, 0.0);

    const double theta = NormalizeAngleDeg(
        std::atan2(P_world[1] - pole_[1], P_world[0] - pole_[0]) * kRadToDeg);
    if (num_cameras_ == 1) {
        weights[sorted_[0].second] = 1.0;
        return weights;
    }

    const double blend_half = ComputeBlendHalfAngle(num_cameras_);

    const int best = NearestCameraIndex(theta);

    if (TryBlend((best - 1 + num_cameras_) % num_cameras_, theta, blend_half, best, weights))
        return weights;
    if (TryBlend(best, theta, blend_half, best, weights))
        return weights;

    weights[sorted_[best].second] = 1.0;
    return weights;
}

} // namespace avm_stitch
