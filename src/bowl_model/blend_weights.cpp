#include "bowl_model/blend_weights.hpp"

#include "common/angle_utils.hpp"

#include <algorithm>
#include <cmath>
#include <span>
#include <utility>

namespace avm_stitch {

namespace {

double ShortestArcDeg(double angle_a_deg, double angle_b_deg) {
    double distance_deg = std::abs(NormalizeAngleDeg(angle_a_deg - angle_b_deg));
    if (distance_deg > 180.0)
        distance_deg = 360.0 - distance_deg;
    return distance_deg;
}

double SectorBoundaryDeg(double azimuth_a_deg, double azimuth_b_deg) {
    if (azimuth_b_deg < azimuth_a_deg)
        azimuth_b_deg += 360.0;
    return NormalizeAngleDeg((azimuth_a_deg + azimuth_b_deg) / 2.0);
}

} // namespace

BlendWeightsCalculator::BlendWeightsCalculator(
    const std::vector<Camera>& cameras, cv::Vec2d pole)
    : pole_(pole) {
    num_cameras_ = static_cast<int>(cameras.size());
    cameras_by_azimuth_.reserve(num_cameras_);
    for (size_t i = 0; i < cameras.size(); ++i)
        cameras_by_azimuth_.emplace_back(cameras[i].Extrinsics().AzimuthDeg(),
                                         static_cast<int>(i));
    std::sort(cameras_by_azimuth_.begin(), cameras_by_azimuth_.end());
}

double BlendWeightsCalculator::ComputeBlendHalfAngle(int num_cameras) {
    if (num_cameras <= 0)
        return 0.0;
    return 52.0 / num_cameras;
}

int BlendWeightsCalculator::NearestCameraIndex(double theta_deg) const {
    int nearest_index = 0;
    double best_dist = 360.0;
    for (int camera_index = 0; camera_index < num_cameras_; ++camera_index) {
        const double distance_deg =
            ShortestArcDeg(theta_deg, cameras_by_azimuth_[camera_index].first);
        if (distance_deg < best_dist) {
            best_dist = distance_deg;
            nearest_index = camera_index;
        }
    }
    return nearest_index;
}

bool BlendWeightsCalculator::TryBlend(
    int near_index, double theta_deg, double blend_half_deg,
    int nearest_index, std::vector<double>& weights) const {
    const int next_index = (near_index + 1) % num_cameras_;
    const double boundary = SectorBoundaryDeg(
        cameras_by_azimuth_[near_index].first,
        cameras_by_azimuth_[next_index].first);
    const double dist = ShortestArcDeg(theta_deg, boundary);
    if (dist > blend_half_deg)
        return false;
    const double blend_ratio = 0.5 + dist / (2.0 * blend_half_deg);
    const double weight_near = 0.5 * (1.0 - std::cos(blend_ratio * kPi));
    const double weight_far = 1.0 - weight_near;
    if (nearest_index == near_index) {
        weights[cameras_by_azimuth_[near_index].second] = weight_near;
        weights[cameras_by_azimuth_[next_index].second] = weight_far;
    } else {
        weights[cameras_by_azimuth_[next_index].second] = weight_near;
        weights[cameras_by_azimuth_[near_index].second] = weight_far;
    }
    return true;
}

std::vector<double>
BlendWeightsCalculator::WeightsFor(const cv::Vec3d& world_position) const {
    if (num_cameras_ == 0)
        return {};
    std::vector<double> weights(num_cameras_, 0.0);

    const double azimuth_deg = NormalizeAngleDeg(
        std::atan2(world_position[1] - pole_[1],
                   world_position[0] - pole_[0]) *
        kRadToDeg);
    if (num_cameras_ == 1) {
        weights[cameras_by_azimuth_[0].second] = 1.0;
        return weights;
    }

    const double blend_half_deg = ComputeBlendHalfAngle(num_cameras_);

    const int nearest_index = NearestCameraIndex(azimuth_deg);

    if (TryBlend((nearest_index - 1 + num_cameras_) % num_cameras_, azimuth_deg,
                 blend_half_deg, nearest_index, weights))
        return weights;
    if (TryBlend(nearest_index, azimuth_deg, blend_half_deg, nearest_index, weights))
        return weights;

    weights[cameras_by_azimuth_[nearest_index].second] = 1.0;
    return weights;
}

void BlendWeightsCalculator::NormalizeWeights(
    std::span<const double> blend_weights, std::span<const double> edge_factors,
    std::span<float> normalized_weights) const {
    double visible_contrib_sum = 0.0;
    double edge_only_sum = 0.0;
    for (int camera_index = 0; camera_index < num_cameras_; ++camera_index) {
        const size_t index = static_cast<size_t>(camera_index);
        if (edge_factors[index] > 0.0) {
            if (blend_weights[index] > 1e-9)
                visible_contrib_sum += blend_weights[index] * edge_factors[index];
            edge_only_sum += edge_factors[index];
        }
    }

    if (visible_contrib_sum > 1e-9) {
        for (int camera_index = 0; camera_index < num_cameras_; ++camera_index) {
            const size_t index = static_cast<size_t>(camera_index);
            if (edge_factors[index] > 0.0 && blend_weights[index] > 1e-9) {
                normalized_weights[index] =
                    static_cast<float>(blend_weights[index] * edge_factors[index] /
                                       visible_contrib_sum);
            }
        }
    } else if (edge_only_sum > 1e-9) {
        for (int camera_index = 0; camera_index < num_cameras_; ++camera_index) {
            const size_t index = static_cast<size_t>(camera_index);
            if (edge_factors[index] > 0.0) {
                normalized_weights[index] =
                    static_cast<float>(edge_factors[index] / edge_only_sum);
            }
        }
    }
}

} // namespace avm_stitch
