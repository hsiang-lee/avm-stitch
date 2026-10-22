
#include <gtest/gtest.h>

#include "depth/ground_depth.hpp"

#include "test_support/depth_test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <utility>
#include <vector>

using namespace avm_stitch;
using namespace avm_stitch::depth;
using avm_stitch::test::MakeLookDownPair;
using avm_stitch::test::MakeStereoPair;

namespace {

SphericalGrid MakeGrid(int w, int h) {
    SphericalGrid g;
    g.width = w;
    g.height = h;
    g.psi_min_deg = 0.0;
    g.psi_max_deg = 180.0;
    g.mirror_rows = false;
    return g;
}

}
TEST(GroundDepthTest, GroundDepthKnownCase) {
    auto [a, b] = MakeLookDownPair();
    auto basis = MakeRectificationBasis(a, b);
    SphericalGrid g = MakeGrid(33, 33);

    auto map = ComputeGroundDepthMap(basis, a, b, g);
    EXPECT_EQ(1, map.valid(16, 16));
    EXPECT_NEAR(1.0, map.depth(16, 16), 1e-9);
}

TEST(GroundDepthTest, DisparityMonotonicWithDepth) {
    auto [a, b] = MakeStereoPair(/*elevation=*/1.5);
    auto basis = MakeRectificationBasis(a, b);
    SphericalGrid g = MakeGrid(33, 33);

    auto map = ComputeGroundDepthMap(basis, a, b, g);
    const int col = 16;
    std::vector<std::pair<float, float>> samples;
    for (int row = 0; row < g.height; ++row) {
        if (map.valid(row, col)) {
            samples.emplace_back(map.depth(row, col), map.disparity(row, col));
        }
    }
    ASSERT_GE(samples.size(), 5u);
    std::sort(samples.begin(), samples.end(),
              [](const auto& x, const auto& y) { return x.first < y.first; });
    for (size_t i = 1; i < samples.size(); ++i) {
        EXPECT_LT(samples[i].second, samples[i - 1].second + 1e-6);
    }
}

TEST(GroundDepthTest, TriangulateDepthRoundTrip) {
    const cv::Vec3d ta(0.0, 0.0, 1.5);
    const cv::Vec3d tb(1.1, 0.0, 1.5);
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> xy(-5.0, 5.0);
    std::uniform_real_distribution<double> zf(0.5, 5.0);

    for (int i = 0; i < 20; ++i) {
        cv::Vec3d P(xy(rng), xy(rng), zf(rng));
        cv::Vec3d da = P - ta;
        da /= cv::norm(da);
        cv::Vec3d db = P - tb;
        db /= cv::norm(db);

        double lambda = TriangulateDepth(da, ta, db, tb);
        cv::Vec3d rec = ta + lambda * da;
        EXPECT_NEAR(P[0], rec[0], 1e-9);
        EXPECT_NEAR(P[1], rec[1], 1e-9);
        EXPECT_NEAR(P[2], rec[2], 1e-9);
    }
}

TEST(GroundDepthTest, InvalidOutsideRange) {
    auto [a, b] = MakeLookDownPair();
    auto basis = MakeRectificationBasis(a, b);
    SphericalGrid g = MakeGrid(33, 33);

    auto map = ComputeGroundDepthMap(basis, a, b, g);
    EXPECT_EQ(0, map.valid(32, 16));
    EXPECT_EQ(0.0f, map.depth(32, 16));
}

TEST(GroundDepthTest, MapShapes) {
    auto [a, b] = MakeLookDownPair();
    auto basis = MakeRectificationBasis(a, b);
    SphericalGrid g = MakeGrid(64, 32);

    auto map = ComputeGroundDepthMap(basis, a, b, g);
    EXPECT_EQ(32, map.depth.rows);
    EXPECT_EQ(64, map.depth.cols);
    EXPECT_EQ(CV_32F, map.depth.type());
    EXPECT_EQ(CV_32F, map.disparity.type());
    EXPECT_EQ(CV_8U, map.valid.type());
}

TEST(GroundDepthTest, DisparityAnalyticValue) {
    //
    auto [a, b] = MakeLookDownPair();
    auto basis = MakeRectificationBasis(a, b);
    SphericalGrid g = MakeGrid(33, 33);

    auto map = ComputeGroundDepthMap(basis, a, b, g);
    struct RowDisparity {
        int row;
        float expect;
    };
    const RowDisparity kRows[] = {
        {9, 1.962523f},
        {10, 3.722848f},
        {11, 5.165377f},
        {12, 6.269225f},
        {13, 7.065332f},
        {14, 7.597193f},
        {15, 7.901194f},
        {16, 8.000000f},
        {17, 7.901194f},
        {18, 7.597193f},
        {19, 7.065332f},
        {20, 6.269225f},
        {21, 5.165377f},
        {22, 3.722848f},
        {23, 1.962523f},
    };
    const int col = 16;
    for (const auto& r : kRows) {
        EXPECT_EQ(1, map.valid(r.row, col)) << "row " << r.row;
        EXPECT_NEAR(r.expect, map.disparity(r.row, col), 1e-3) << "row " << r.row;
    }
}

TEST(GroundDepthTest, DepthRangeBoundsInvalid) {
    auto [a, b] = MakeLookDownPair();
    auto basis = MakeRectificationBasis(a, b);
    SphericalGrid g = MakeGrid(33, 33);
    GroundDepthOptions opts;
    opts.min_depth = 2.0;
    opts.max_depth = 3.0;

    auto map = ComputeGroundDepthMap(basis, a, b, g, opts);
    const int col = 16;

    EXPECT_EQ(1, map.valid(10, col));
    EXPECT_NEAR(2.6131, map.depth(10, col), 1e-3);
    EXPECT_EQ(1, map.valid(22, col));
    EXPECT_NEAR(2.6131, map.depth(22, col), 1e-3);

    EXPECT_EQ(0, map.valid(16, col));
    EXPECT_EQ(0.0f, map.depth(16, col));

    EXPECT_EQ(0, map.valid(9, col));
    EXPECT_EQ(0.0f, map.depth(9, col));
    EXPECT_EQ(0, map.valid(23, col));
    EXPECT_EQ(0.0f, map.depth(23, col));
}
