
#include <gtest/gtest.h>

#include "bowl_model/blend_weights.hpp"
#include "camera_model/camera_extrinsics.hpp"

#include "test_support/geometry_test_helpers.hpp"
#include "test_support/stitch_test_helpers.hpp"

#include <cmath>
#include <vector>

using namespace avm_stitch;
using avm_stitch::test::DefaultBakeCameras;
using avm_stitch::test::MakeSideCameras;

TEST(BlendWeightsTest, DefaultPole_BehaviorUnchanged) {
    auto cams = DefaultBakeCameras();
    const BlendWeightsCalculator calc_default(cams);
    const BlendWeightsCalculator calc_explicit(cams, cv::Vec2d(0.0, 0.0));
    auto weights_default = calc_default.WeightsFor(cv::Vec3d(5, 0, 0));
    auto weights_explicit = calc_explicit.WeightsFor(cv::Vec3d(5, 0, 0));
    ASSERT_EQ(weights_default.size(), weights_explicit.size());
    for (size_t i = 0; i < weights_default.size(); ++i) {
        EXPECT_NEAR(weights_default[i], weights_explicit[i], 1e-9);
    }
}

TEST(BlendWeightsTest, NonDefaultPole_ShiftsOrigin) {
    auto cams = DefaultBakeCameras();
    const BlendWeightsCalculator calc(cams, cv::Vec2d(2.0, 0.0));
    auto weights = calc.WeightsFor(cv::Vec3d(2, 1, 0));
    ASSERT_EQ(4u, weights.size());
    EXPECT_GT(weights[2], 0.9);
    EXPECT_LT(weights[0], 0.1);
}

TEST(BlendWeightsTest, N4Standard_Front) {
    auto cams = DefaultBakeCameras();
    const BlendWeightsCalculator calc(cams);
    auto weights = calc.WeightsFor(cv::Vec3d(5, 0, 0));
    ASSERT_EQ(4u, weights.size());
    EXPECT_GT(weights[0], 0.9);
    EXPECT_LT(weights[1], 0.1);
}

TEST(BlendWeightsTest, N4Standard_Rear) {
    auto cams = DefaultBakeCameras();
    const BlendWeightsCalculator calc(cams);
    auto weights = calc.WeightsFor(cv::Vec3d(-5, 0, 0));
    EXPECT_GT(weights[1], 0.9);
    EXPECT_LT(weights[0], 0.1);
}

TEST(BlendWeightsTest, N4Standard_BlendBandWidth) {
    double blend = BlendWeightsCalculator::ComputeBlendHalfAngle(4);
    EXPECT_NEAR(13.0, blend, 1e-9);
}

TEST(BlendWeightsTest, N4_BlendZone) {
    auto cams = DefaultBakeCameras();
    const BlendWeightsCalculator calc(cams);
    auto weights = calc.WeightsFor(cv::Vec3d(5, 5, 0));
    int positive = 0;
    double sum = 0.0;
    for (double w : weights) {
        if (w > 0.0)
            positive++;
        sum += w;
    }
    EXPECT_EQ(2, positive);
    EXPECT_NEAR(1.0, sum, 1e-6);
}

TEST(BlendWeightsTest, N4_CosSmoothAtBoundary) {
    auto cams = DefaultBakeCameras();
    const BlendWeightsCalculator calc(cams);
    auto weights = calc.WeightsFor(cv::Vec3d(5, 5, 0));
    EXPECT_NEAR(0.5, weights[0], 1e-6);
    EXPECT_NEAR(0.5, weights[2], 1e-6);
    EXPECT_NEAR(0.0, weights[1], 1e-6);
    EXPECT_NEAR(0.0, weights[3], 1e-6);
}

TEST(BlendWeightsTest, N4_OutsideBlendBand) {
    auto cams = DefaultBakeCameras();
    const BlendWeightsCalculator calc(cams);
    auto weights = calc.WeightsFor(cv::Vec3d(std::cos(65.0 * M_PI / 180.0),
                                             std::sin(65.0 * M_PI / 180.0),
                                             0));
    int ones = 0;
    for (double w : weights) {
        if (w > 0.999)
            ones++;
    }
    EXPECT_EQ(1, ones);
}

TEST(BlendWeightsTest, N8_BlendHalfAngle) {
    double blend = BlendWeightsCalculator::ComputeBlendHalfAngle(8);
    EXPECT_NEAR(6.5, blend, 1e-9);
}

TEST(BlendWeightsTest, N8_WeightsSumToOne) {
    auto cams = MakeSideCameras({0, 45, 90, 135, 180, 225, 270, 315});
    const BlendWeightsCalculator calc(cams);
    std::vector<cv::Vec3d> points = {
        {5, 0, 0}, {0, 5, 0}, {-5, 0, 0}, {0, -5, 0}, {5, 5, 0}, {-5, 5, 0}, {-5, -5, 0}, {5, -5, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}, {-1, 1, 0}};
    for (const auto& P : points) {
        auto weights = calc.WeightsFor(P);
        double sum = 0.0;
        for (double w : weights)
            sum += w;
        EXPECT_NEAR(1.0, sum, 1e-6);
    }
}

TEST(BlendWeightsTest, N3_ArbitrarySpacing) {
    auto cams = MakeSideCameras({0, 120, 240});
    const BlendWeightsCalculator calc(cams);

    std::vector<cv::Vec3d> points = {
        {5, 0, 0}, {0, 5, 0}, {-5, 0, 0}, {0, -5, 0}, {5, 5, 0}};
    for (const auto& P : points) {
        auto weights = calc.WeightsFor(P);
        double sum = 0.0;
        for (double w : weights)
            sum += w;
        EXPECT_NEAR(1.0, sum, 1e-6);
    }
}

TEST(BlendWeightsTest, AllWeightsNonNegative) {
    auto cams = DefaultBakeCameras();
    const BlendWeightsCalculator calc(cams);
    std::vector<cv::Vec3d> points = {
        {5, 0, 0}, {0, 5, 0}, {-5, 0, 0}, {0, -5, 0}, {5, 5, 0}, {-5, 5, 0}, {-5, -5, 0}, {5, -5, 0}};
    for (const auto& P : points) {
        auto weights = calc.WeightsFor(P);
        for (double w : weights)
            EXPECT_GE(w, 0.0);
    }
}

TEST(BlendWeightsTest, N1_SingleCamera) {
    auto cams = MakeSideCameras({0});
    const BlendWeightsCalculator calc(cams);
    auto weights = calc.WeightsFor(cv::Vec3d(5, 0, 0));
    ASSERT_EQ(1u, weights.size());
    EXPECT_NEAR(1.0, weights[0], 1e-9);
}

TEST(BlendWeightsTest, EmptyCameras_ReturnsEmpty) {
    BlendWeightsCalculator calc({});
    auto w = calc.WeightsFor(cv::Vec3d(0.0, 0.0, 0.0));
    EXPECT_TRUE(w.empty());
}
