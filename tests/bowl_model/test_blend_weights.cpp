
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
    auto weights_default = ComputeBlendWeights(cv::Vec3d(5, 0, 0), cams);
    auto weights_explicit = ComputeBlendWeights(cv::Vec3d(5, 0, 0), cams,
                                                cv::Vec2d(0.0, 0.0));
    ASSERT_EQ(weights_default.size(), weights_explicit.size());
    for (size_t i = 0; i < weights_default.size(); ++i) {
        EXPECT_NEAR(weights_default[i], weights_explicit[i], 1e-9);
    }
}

TEST(BlendWeightsTest, NonDefaultPole_ShiftsOrigin) {
        auto cams = DefaultBakeCameras();
    auto weights = ComputeBlendWeights(cv::Vec3d(2, 1, 0), cams,
                                       cv::Vec2d(2.0, 0.0));
    ASSERT_EQ(4u, weights.size());
        EXPECT_GT(weights[2], 0.9);
    EXPECT_LT(weights[0], 0.1);
}


TEST(BlendWeightsTest, N4Standard_Front) {
    auto cams = DefaultBakeCameras();
    auto weights = ComputeBlendWeights(cv::Vec3d(5, 0, 0), cams);
    ASSERT_EQ(4u, weights.size());
    EXPECT_GT(weights[0], 0.9);       EXPECT_LT(weights[1], 0.1);   }

TEST(BlendWeightsTest, N4Standard_Rear) {
    auto cams = DefaultBakeCameras();
    auto weights = ComputeBlendWeights(cv::Vec3d(-5, 0, 0), cams);
    EXPECT_GT(weights[1], 0.9);       EXPECT_LT(weights[0], 0.1);
}

TEST(BlendWeightsTest, N4Standard_BlendBandWidth) {
    double blend = ComputeBlendHalfAngle(4);
    EXPECT_NEAR(13.0, blend, 1e-9);   }

TEST(BlendWeightsTest, N4_BlendZone) {
        auto cams = DefaultBakeCameras();
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
        auto cams = DefaultBakeCameras();
    auto weights = ComputeBlendWeights(cv::Vec3d(5, 5, 0), cams);
    EXPECT_NEAR(0.5, weights[0], 1e-6);       EXPECT_NEAR(0.5, weights[2], 1e-6);       EXPECT_NEAR(0.0, weights[1], 1e-6);       EXPECT_NEAR(0.0, weights[3], 1e-6);   }

TEST(BlendWeightsTest, N4_OutsideBlendBand) {
        auto cams = DefaultBakeCameras();
    auto weights = ComputeBlendWeights(cv::Vec3d(std::cos(65.0 * M_PI / 180.0),
                                                 std::sin(65.0 * M_PI / 180.0),
                                                 0),
                                       cams);
    int ones = 0;
    for (double w : weights) {
        if (w > 0.999) ones++;
    }
    EXPECT_EQ(1, ones);   }


TEST(BlendWeightsTest, N8_BlendHalfAngle) {
    double blend = ComputeBlendHalfAngle(8);
    EXPECT_NEAR(6.5, blend, 1e-9);   }

TEST(BlendWeightsTest, N8_WeightsSumToOne) {
    auto cams = MakeSideCameras({0, 45, 90, 135, 180, 225, 270, 315});
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


TEST(BlendWeightsTest, N3_ArbitrarySpacing) {
    auto cams = MakeSideCameras({0, 120, 240});

    std::vector<cv::Vec3d> points = {
        {5, 0, 0}, {0, 5, 0}, {-5, 0, 0}, {0, -5, 0}, {5, 5, 0}
    };
    for (const auto& P : points) {
        auto weights = ComputeBlendWeights(P, cams);
        double sum = 0.0;
        for (double w : weights) sum += w;
        EXPECT_NEAR(1.0, sum, 1e-6);       }
}

TEST(BlendWeightsTest, AllWeightsNonNegative) {
    auto cams = DefaultBakeCameras();
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
    auto cams = MakeSideCameras({0});
    auto weights = ComputeBlendWeights(cv::Vec3d(5, 0, 0), cams);
    ASSERT_EQ(1u, weights.size());
    EXPECT_NEAR(1.0, weights[0], 1e-9);
}
