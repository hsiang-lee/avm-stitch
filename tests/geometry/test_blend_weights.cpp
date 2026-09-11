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
// R_wc is constructed so the optical axis (camera Z in world) points in the
// azimuth direction: forward = (cos(az), sin(az), 0).
// Camera Y (down) = (0, 0, -1). Right = down × forward (right-hand rule).
std::vector<CameraExtrinsics> MakeNCameras(
    const std::vector<double>& azimuths_deg) {
    std::vector<CameraExtrinsics> cams;
    for (double az : azimuths_deg) {
        double rad = az * M_PI / 180.0;
        cv::Vec3d pos(std::cos(rad) * 3.0, std::sin(rad) * 3.0, 1.0);
        // Build R_wc: column 2 = forward (optical axis), column 1 = down, column 0 = right
        cv::Vec3d forward(std::cos(rad), std::sin(rad), 0.0);
        cv::Vec3d down(0.0, 0.0, -1.0);
        cv::Vec3d right = down.cross(forward);
        cv::Matx33d R(right[0], down[0], forward[0],
                      right[1], down[1], forward[1],
                      right[2], down[2], forward[2]);
        cams.push_back({R, pos});
    }
    return cams;
}

} // namespace

// ---------------------------------------------------------------------------
// Optical-axis azimuth tests (Task 2: D2 sector model — AzimuthDeg uses
// optical axis ground projection, not camera position azimuth)
// ---------------------------------------------------------------------------

TEST(BlendWeightsTest, OpticalAxisAzimuth_Front) {
    // From examples/data/cameras.json front camera
    // R column 2 (optical axis in world) = (0.9623, 0.0319, -0.2700)
    // Ground projection: atan2(0.0319, 0.9623) ≈ 1.9° → ~2°
    CameraExtrinsics cam;
    cam.R = cv::Matx33d(0.021611199265347603, -0.27100261158445849, 0.96233598113170238,
                        -0.99892382596379004, 0.033658009819147572, 0.031911256585685610,
                        -0.041038347775912420, -0.96198998065957897, -0.26998357565304970);
    cam.t = cv::Vec3d(4.1166681677199062, -0.053393489601802596, 0.68738271643240068);
    auto boundaries = ComputeSectorBoundaries({cam});
    ASSERT_EQ(1u, boundaries.size());
    // Old formula (atan2(t[1], t[0])) gives ~359°, new should give ~2°
    EXPECT_NEAR(2.0, boundaries[0], 1.0);
}

TEST(BlendWeightsTest, OpticalAxisAzimuth_Rear) {
    // R column 2 = (-0.9610, 0.0021, -0.2766)
    // Ground projection: atan2(0.0021, -0.9610) ≈ 180°
    CameraExtrinsics cam;
    cam.R = cv::Matx33d(-0.0017209126782142814, 0.27659367005804369, -0.96098542140002097,
                        0.99990614312893356, 0.013537735644589266, 0.0021058600707276054,
                        0.013592034158808939, -0.96089160231393378, -0.27659100710252038);
    cam.t = cv::Vec3d(-1.0661498792051314, 0.027999355244672404, 1.0483336211871681);
    auto boundaries = ComputeSectorBoundaries({cam});
    ASSERT_EQ(1u, boundaries.size());
    EXPECT_NEAR(180.0, boundaries[0], 1.0);
}

TEST(BlendWeightsTest, OpticalAxisAzimuth_Left) {
    // R column 2 = (-0.0259, 0.9244, -0.3806)
    // Ground projection: atan2(0.9244, -0.0259) ≈ 92°
    CameraExtrinsics cam;
    cam.R = cv::Matx33d(0.99951987693407285, -0.017004553886587667, -0.025900979919959218,
                        0.017470826282207755, -0.38107046696496416, 0.92438091144079837,
                        -0.025588783533277093, -0.92438960636423340, -0.38059042263707160);
    cam.t = cv::Vec3d(2.4621261743376248, 1.0957842556321686, 1.2309507669565125);
    auto boundaries = ComputeSectorBoundaries({cam});
    ASSERT_EQ(1u, boundaries.size());
    EXPECT_NEAR(92.0, boundaries[0], 1.0);
}

TEST(BlendWeightsTest, OpticalAxisAzimuth_Right) {
    // R column 2 = (-0.0102, -0.9613, -0.2753)
    // Ground projection: atan2(-0.9613, -0.0102) ≈ -90.6° → NormalizeAngleDeg → ~269°
    CameraExtrinsics cam;
    cam.R = cv::Matx33d(-0.99898909342767439, -0.043787710798984310, -0.010169935866938002,
                        -0.0022889888374402548, 0.27548936002331326, -0.96130139552798322,
                        0.044894896621500230, -0.96030633075958138, -0.27531109559989531);
    cam.t = cv::Vec3d(2.4741283790509305, -0.94232713150490754, 1.1323086168468555);
    auto boundaries = ComputeSectorBoundaries({cam});
    ASSERT_EQ(1u, boundaries.size());
    // NormalizeAngleDeg(-90.6°) = 269.4° → ~269°
    EXPECT_NEAR(269.0, boundaries[0], 1.0);
}

// ---------------------------------------------------------------------------
// Pole parameter tests (Task 3: ComputeBlendWeights pole argument)
// ---------------------------------------------------------------------------

TEST(BlendWeightsTest, DefaultPole_BehaviorUnchanged) {
    // Verify that default pole=(0,0) produces same result as explicit (0,0).
    // (Requires MakeNCameras with proper rotation — Task 4; at Task 3 level
    // this test verifies the new overload compiles and self-consistency.)
    auto cams = MakeNCameras({0, 180, 90, -90});
    auto weights_default = ComputeBlendWeights(cv::Vec3d(5, 0, 0), cams);
    auto weights_explicit = ComputeBlendWeights(cv::Vec3d(5, 0, 0), cams,
                                                cv::Vec2d(0.0, 0.0));
    ASSERT_EQ(weights_default.size(), weights_explicit.size());
    for (size_t i = 0; i < weights_default.size(); ++i) {
        EXPECT_NEAR(weights_default[i], weights_explicit[i], 1e-9);
    }
}

TEST(BlendWeightsTest, NonDefaultPole_ShiftsOrigin) {
    // P=(2,1,0), pole=(2,0) → relative azimuth = atan2(1, 0) = 90°
    auto cams = MakeNCameras({0, 180, 90, -90});
    auto weights = ComputeBlendWeights(cv::Vec3d(2, 1, 0), cams,
                                       cv::Vec2d(2.0, 0.0));
    ASSERT_EQ(4u, weights.size());
    // At 90° azimuth, left camera (index 2) should dominate
    EXPECT_GT(weights[2], 0.9);
    EXPECT_LT(weights[0], 0.1);
}

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
