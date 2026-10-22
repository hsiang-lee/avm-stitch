#pragma once

#include "camera_model/camera.hpp"
#include "common/angle_utils.hpp"

#include <opencv2/core.hpp>
#include <span>
#include <utility>
#include <vector>

namespace avm_stitch {

class BlendWeightsCalculator {
  public:
    explicit BlendWeightsCalculator(const std::vector<Camera>& cameras,
                                    cv::Vec2d pole = cv::Vec2d(0.0, 0.0));
    std::vector<double> WeightsFor(const cv::Vec3d& world_position) const;
    void NormalizeWeights(std::span<const double> blend_weights,
                          std::span<const double> edge_factors,
                          std::span<float> normalized_weights) const;
    static double ComputeBlendHalfAngle(int num_cameras);

  private:
    int NearestCameraIndex(double theta_deg) const;
    bool TryBlend(int near_index, double theta_deg, double blend_half_deg,
                  int nearest_index, std::vector<double>& weights) const;
    std::vector<std::pair<double, int>> cameras_by_azimuth_;
    cv::Vec2d pole_;
    int num_cameras_ = 0;
};

} // namespace avm_stitch
