#include "depth/ground_depth.hpp"
#include "depth/depth_common.hpp"

#include <cmath>

namespace avm_stitch::depth {

namespace {

using detail::kDegToRad;

struct GroundSample {
    float depth = 0.0f;           float disparity = 0.0f;   };

bool SampleGroundCell(const RectificationBasis& basis,
                      const CameraExtrinsics& a, const CameraExtrinsics& b,
                      const SphericalGrid& grid, double psi0, double psi1,
                      int row, int col, const GroundDepthOptions& opts,
                      GroundSample& out) {
        cv::Vec3d d_r = grid.DirectionC(row, col);
    cv::Vec3d d_a = CameraDirection(basis, d_r, false);
    cv::Vec3d d_world = a.RotateCameraToWorld(d_a);

            const double denom = d_world[2];
    if (std::abs(denom) < 1e-12) return false;
    const double lambda = (opts.ground_z - a.position()[2]) / denom;
    if (!std::isfinite(lambda) || lambda < opts.min_depth || lambda > opts.max_depth) {
        return false;
    }
    const cv::Vec3d P = a.position() + lambda * d_world;

        cv::Vec3d d_b_world = P - b.position();
    d_b_world /= cv::norm(d_b_world);
    cv::Vec3d d_b = b.RotateWorldToCamera(d_b_world);
    cv::Vec3d d_rb = basis.m2 * d_b;

            double psi_b = std::atan2(std::hypot(d_rb[1], d_rb[2]), d_rb[0]);
    if (psi_b < psi0 || psi_b > psi1) return false;
    double col_b = (psi_b - psi0) / (psi1 - psi0) * (grid.width - 1);

    out.depth = static_cast<float>(lambda);
    out.disparity = static_cast<float>(col_b - col);
    return true;
}

}
GroundDepthMap ComputeGroundDepthMap(const RectificationBasis& basis,
                                     const CameraExtrinsics& a,
                                     const CameraExtrinsics& b,
                                     const SphericalGrid& grid,
                                     const GroundDepthOptions& opts) {
    GroundDepthMap out;
    const int h = grid.height, w = grid.width;
    out.depth = cv::Mat1f::zeros(h, w);
    out.disparity = cv::Mat1f::zeros(h, w);
    out.valid = cv::Mat1b::zeros(h, w);

    const double psi0 = grid.psi_min_deg * kDegToRad;       const double psi1 = grid.psi_max_deg * kDegToRad;
    if (w < 2 || h < 1 || psi1 <= psi0) return out;

    for (int row = 0; row < h; ++row) {
        for (int col = 0; col < w; ++col) {
            GroundSample s;
            if (!SampleGroundCell(basis, a, b, grid, psi0, psi1, row, col, opts, s)) {
                continue;
            }
            out.depth(row, col) = s.depth;
            out.disparity(row, col) = s.disparity;
            out.valid(row, col) = 1;
        }
    }
    return out;
}

double TriangulateDepth(const cv::Vec3d& dir_a_world, const cv::Vec3d& origin_a,
                        const cv::Vec3d& dir_b_world, const cv::Vec3d& origin_b) {
                            const double aa = dir_a_world.dot(dir_a_world);
    const double bb = dir_b_world.dot(dir_b_world);
    const double ab = dir_a_world.dot(dir_b_world);
    const double det = aa * bb - ab * ab;
    if (std::abs(det) < 1e-12) return 0.0;
    const cv::Vec3d C = origin_b - origin_a;
    const double r0 = dir_a_world.dot(C);
    const double r1 = -dir_b_world.dot(C);
        return (bb * r0 + ab * r1) / det;
}

}
