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

    int best = 0;
    double best_dist = 360.0;
    for (int j = 0; j < num_cameras_; ++j) {
        double d = ShortestArcDeg(theta, sorted_[j].first);
        if (d < best_dist) {
            best_dist = d;
            best = j;
        }
    }

    auto try_blend = [&](int a) -> bool {
        const int b = (a + 1) % num_cameras_;
        const double boundary = SectorBoundaryDeg(sorted_[a].first, sorted_[b].first);
        const double dist = ShortestArcDeg(theta, boundary);
        if (dist > blend_half)
            return false;
        const double t = 0.5 + dist / (2.0 * blend_half);
        const double w_near = 0.5 * (1.0 - std::cos(t * kPi));
        const double w_far = 1.0 - w_near;
        if (best == a) {
            weights[sorted_[a].second] = w_near;
            weights[sorted_[b].second] = w_far;
        } else {
            weights[sorted_[b].second] = w_near;
            weights[sorted_[a].second] = w_far;
        }
        return true;
    };

    if (try_blend((best - 1 + num_cameras_) % num_cameras_))
        return weights;
    if (try_blend(best))
        return weights;

    weights[sorted_[best].second] = 1.0;
    return weights;
}

} // namespace avm_stitch
