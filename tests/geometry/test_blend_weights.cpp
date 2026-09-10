// Tests for avm_stitch N-camera blend weights (azimuth sectors + cosine blend).
// Pure math, no GL / OSG dependency.

#include <gtest/gtest.h>

#include "geometry/blend_weights.hpp"
#include "geometry/projection.hpp"

#include <cmath>
#include <vector>

using namespace avm_stitch;

namespace {

// N cameras at given azimuths (degrees) around the origin, radius 3m, height 1m.
// Rotation is identity; blend weights only depend on camera position azimuth.
std::vector<CameraExtrinsics> MakeNCameras(
    const std::vector<double>& azimuths_deg) {
    std::vector<CameraExtrinsics> cams;
    for (double az : azimuths_deg) {
        double rad = az * M_PI / 180.0;
        cv::Vec3d pos(std::cos(rad) * 3.0, std::sin(rad) * 3.0, 1.0);
        cams.push_back({cv::Matx33d::eye(), pos});
    }
    return cams;
}

} // namespace

// ---------------------------------------------------------------------------
// N=4 standard layout: front(0), rear(180), left(90), right(-90)
// ---------------------------------------------------------------------------

TEST(BlendWeightsTest, N4Standard_Front) {
    auto cams = MakeNCameras({0, 180, 90, -90});
    auto weights = ComputeBlendWeights(cv::Vec3d(5, 0, 0), cams);
    ASSERT_EQ(4u, weights.size());
    EXPECT_GT(weights[0], 0.9);   // front dominant
    EXPECT_LT(weights[1], 0.1);   // rear
}

TEST(BlendWeightsTest, N4Standard_Rear) {
    auto cams = MakeNCameras({0, 180, 90, -90});
    auto weights = ComputeBlendWeights(cv::Vec3d(-5, 0, 0), cams);
    EXPECT_GT(weights[1], 0.9);   // rear dominant
    EXPECT_LT(weights[0], 0.1);
}

TEST(BlendWeightsTest, N4Standard_SectorBoundaries) {
    auto cams = MakeNCameras({0, 180, 90, -90});
    auto result = ComputeSectorBoundaries(cams);
    ASSERT_EQ(4u, result.size());
    EXPECT_NEAR(45.0, result[0], 1.0);
    EXPECT_NEAR(135.0, result[1], 1.0);
    EXPECT_NEAR(225.0, result[2], 1.0);
    EXPECT_NEAR(315.0, result[3], 1.0);
}

TEST(BlendWeightsTest, N4Standard_BlendBandWidth) {
    double blend = ComputeBlendHalfAngle(4);
    EXPECT_NEAR(13.0, blend, 1e-9);   // 52 / 4
}

TEST(BlendWeightsTest, N4_BlendZone) {
    // P at 45 deg azimuth -> exactly on the front/left sector boundary.
    auto cams = MakeNCameras({0, 180, 90, -90});
    auto weights = ComputeBlendWeights(cv::Vec3d(5, 5, 0), cams);
    int positive = 0;
    double sum = 0.0;
    for (double w : weights) {
        if (w > 0.0) positive++;
        sum += w;
    }
    EXPECT_EQ(2, positive);
    EXPECT_NEAR(1.0, sum, 1e-6);
}

TEST(BlendWeightsTest, N4_CosSmoothAtBoundary) {
    // Exactly on the boundary -> the two adjacent cameras split 50/50.
    auto cams = MakeNCameras({0, 180, 90, -90});
    auto weights = ComputeBlendWeights(cv::Vec3d(5, 5, 0), cams);
    EXPECT_NEAR(0.5, weights[0], 1e-6);   // front
    EXPECT_NEAR(0.5, weights[2], 1e-6);   // left
    EXPECT_NEAR(0.0, weights[1], 1e-6);   // rear
    EXPECT_NEAR(0.0, weights[3], 1e-6);   // right
}

TEST(BlendWeightsTest, N4_OutsideBlendBand) {
    // 65 deg azimuth: 20 deg past the 45 deg boundary (blend_half = 13 deg).
    auto cams = MakeNCameras({0, 180, 90, -90});
    auto weights = ComputeBlendWeights(cv::Vec3d(std::cos(65.0 * M_PI / 180.0),
                                                 std::sin(65.0 * M_PI / 180.0),
                                                 0),
                                       cams);
    int ones = 0;
    for (double w : weights) {
        if (w > 0.999) ones++;
    }
    EXPECT_EQ(1, ones);   // single camera owns the point
}

// ---------------------------------------------------------------------------
// N=8 uniform layout
// ---------------------------------------------------------------------------

TEST(BlendWeightsTest, N8_SectorWidth45) {
    auto cams = MakeNCameras({0, 45, 90, 135, 180, 225, 270, 315});
    auto result = ComputeSectorBoundaries(cams);
    ASSERT_EQ(8u, result.size());
    double expected = 22.5;
    for (size_t j = 0; j < result.size(); ++j) {
        EXPECT_NEAR(expected, result[j], 1.0);
        expected += 45.0;
    }
}

TEST(BlendWeightsTest, N8_BlendHalfAngle) {
    double blend = ComputeBlendHalfAngle(8);
    EXPECT_NEAR(6.5, blend, 1e-9);   // 52 / 8
}

TEST(BlendWeightsTest, N8_WeightsSumToOne) {
    auto cams = MakeNCameras({0, 45, 90, 135, 180, 225, 270, 315});
    std::vector<cv::Vec3d> points = {
        {5, 0, 0}, {0, 5, 0}, {-5, 0, 0}, {0, -5, 0},
        {5, 5, 0}, {-5, 5, 0}, {-5, -5, 0}, {5, -5, 0},
        {1, 0, 0}, {0, 1, 0}, {1, 1, 0}, {-1, 1, 0}
    };
    for (const auto& P : points) {
        auto weights = ComputeBlendWeights(P, cams);
        double sum = 0.0;
        for (double w : weights) sum += w;
        EXPECT_NEAR(1.0, sum, 1e-6);
    }
}

// ---------------------------------------------------------------------------
// Other layouts and invariants
// ---------------------------------------------------------------------------

TEST(BlendWeightsTest, N3_ArbitrarySpacing) {
    auto cams = MakeNCameras({0, 120, 240});
    auto result = ComputeSectorBoundaries(cams);
    ASSERT_EQ(3u, result.size());
    EXPECT_NEAR(60.0, result[0], 1.0);
    EXPECT_NEAR(180.0, result[1], 1.0);
    EXPECT_NEAR(300.0, result[2], 1.0);

    std::vector<cv::Vec3d> points = {
        {5, 0, 0}, {0, 5, 0}, {-5, 0, 0}, {0, -5, 0}, {5, 5, 0}
    };
    for (const auto& P : points) {
        auto weights = ComputeBlendWeights(P, cams);
        double sum = 0.0;
        for (double w : weights) sum += w;
        EXPECT_NEAR(1.0, sum, 1e-6);   // no crash, sum preserved
    }
}

TEST(BlendWeightsTest, AllWeightsNonNegative) {
    auto cams = MakeNCameras({0, 180, 90, -90});
    std::vector<cv::Vec3d> points = {
        {5, 0, 0}, {0, 5, 0}, {-5, 0, 0}, {0, -5, 0},
        {5, 5, 0}, {-5, 5, 0}, {-5, -5, 0}, {5, -5, 0}
    };
    for (const auto& P : points) {
        auto weights = ComputeBlendWeights(P, cams);
        for (double w : weights) EXPECT_GE(w, 0.0);
    }
}

TEST(BlendWeightsTest, N1_SingleCamera) {
    auto cams = MakeNCameras({0});
    auto weights = ComputeBlendWeights(cv::Vec3d(5, 0, 0), cams);
    ASSERT_EQ(1u, weights.size());
    EXPECT_NEAR(1.0, weights[0], 1e-9);
}
