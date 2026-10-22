#pragma once
#include "depth/spherical_rectification.hpp"
#include <opencv2/core.hpp>


namespace avm_stitch::depth {

struct GroundDepthOptions {
    double ground_z = 0.0;
    double min_depth = 0.5;
    double max_depth = 100.0;
};

struct GroundDepthMap {
    cv::Mat1f depth;           cv::Mat1f disparity;       cv::Mat1b valid;
};

GroundDepthMap ComputeGroundDepthMap(const RectificationBasis& basis,
                                     const CameraExtrinsics& a,
                                     const CameraExtrinsics& b,
                                     const SphericalGrid& grid,
                                     const GroundDepthOptions& opts = {});

double TriangulateDepth(const cv::Vec3d& dir_a_world, const cv::Vec3d& origin_a,
                        const cv::Vec3d& dir_b_world, const cv::Vec3d& origin_b);

}
