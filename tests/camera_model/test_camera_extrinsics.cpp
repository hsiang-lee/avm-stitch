
#include <gtest/gtest.h>

#include "camera_model/camera_extrinsics.hpp"

#include <opencv2/core.hpp>

#include <cmath>

using namespace avm_stitch;

TEST(CameraExtrinsicsTest, ToCamera_MatchesHandComputedRtPmT) {
            const double s = 0.5, c = std::sqrt(3.0) / 2.0;
    const cv::Matx33d R(c, 0, s,
                        0, 1, 0,
                       -s, 0, c);
    const cv::Vec3d t(1.0, 2.0, -3.0);
    CameraExtrinsics extr{R, t};

    const cv::Vec3d P(4.0, -5.0, 6.0);
    const cv::Vec3d expected = R.t() * (P - t);
    const cv::Vec3d got = extr.ToCamera(P);
    EXPECT_DOUBLE_EQ(expected[0], got[0]);
    EXPECT_DOUBLE_EQ(expected[1], got[1]);
    EXPECT_DOUBLE_EQ(expected[2], got[2]);
}

TEST(CameraExtrinsicsTest, ToWorldRoundTripsToCamera) {
            const double s = 0.5, c = std::sqrt(3.0) / 2.0;
    const cv::Matx33d R(c, 0, s,
                        0, 1, 0,
                       -s, 0, c);
    const cv::Vec3d t(1.0, -2.0, 3.0);
    CameraExtrinsics extr{R, t};

    const cv::Vec3d P(-2.0, 0.5, 4.0);
    const cv::Vec3d back = extr.ToWorld(extr.ToCamera(P));
    EXPECT_NEAR(P[0], back[0], 1e-12);
    EXPECT_NEAR(P[1], back[1], 1e-12);
    EXPECT_NEAR(P[2], back[2], 1e-12);
}

TEST(CameraExtrinsicsTest, ToWorld_MatchesHandComputedRPtT) {
    const double s = 0.5, c = std::sqrt(3.0) / 2.0;
    const cv::Matx33d R(c, 0, s,
                        0, 1, 0,
                       -s, 0, c);
    const cv::Vec3d t(1.0, -2.0, 3.0);
    CameraExtrinsics extr{R, t};

    const cv::Vec3d P_cam(0.3, -0.7, 1.4);
    const cv::Vec3d expected = R * P_cam + t;
    const cv::Vec3d got = extr.ToWorld(P_cam);
    EXPECT_DOUBLE_EQ(expected[0], got[0]);
    EXPECT_DOUBLE_EQ(expected[1], got[1]);
    EXPECT_DOUBLE_EQ(expected[2], got[2]);
}

TEST(CameraExtrinsicsTest, RotateWorldToCamera_MatchesHandComputedRtV) {
    const double s = 0.5, c = std::sqrt(3.0) / 2.0;
    const cv::Matx33d R(c, 0, s,
                        0, 1, 0,
                       -s, 0, c);
    CameraExtrinsics extr{R, cv::Vec3d(0, 0, 0)};

    const cv::Vec3d v(0.5, -0.25, 0.75);
    const cv::Vec3d expected = R.t() * v;
    const cv::Vec3d got = extr.RotateWorldToCamera(v);
    EXPECT_DOUBLE_EQ(expected[0], got[0]);
    EXPECT_DOUBLE_EQ(expected[1], got[1]);
    EXPECT_DOUBLE_EQ(expected[2], got[2]);
}

TEST(CameraExtrinsicsTest, RotateCameraToWorld_MatchesHandComputedRv) {
    const double s = 0.5, c = std::sqrt(3.0) / 2.0;
    const cv::Matx33d R(c, 0, s,
                        0, 1, 0,
                       -s, 0, c);
    CameraExtrinsics extr{R, cv::Vec3d(0, 0, 0)};

    const cv::Vec3d v(0.5, -0.25, 0.75);
    const cv::Vec3d expected = R * v;
    const cv::Vec3d got = extr.RotateCameraToWorld(v);
    EXPECT_DOUBLE_EQ(expected[0], got[0]);
    EXPECT_DOUBLE_EQ(expected[1], got[1]);
    EXPECT_DOUBLE_EQ(expected[2], got[2]);
}

TEST(CameraExtrinsicsTest, Position_MatchesTVector) {
    const double s = 0.5, c = std::sqrt(3.0) / 2.0;
    const cv::Matx33d R(c, 0, s,
                        0, 1, 0,
                       -s, 0, c);
    const cv::Vec3d t(1.0, 2.0, -3.0);
    CameraExtrinsics extr{R, t};

    EXPECT_DOUBLE_EQ(t[0], extr.position()[0]);
    EXPECT_DOUBLE_EQ(t[1], extr.position()[1]);
    EXPECT_DOUBLE_EQ(t[2], extr.position()[2]);
}

TEST(CameraExtrinsicsTest, DefaultConstructor_IsIdentity) {
            CameraExtrinsics extr;
    const cv::Vec3d P(1.0, 2.0, 3.0);
    const cv::Vec3d out = extr.ToCamera(P);
    EXPECT_DOUBLE_EQ(P[0], out[0]);
    EXPECT_DOUBLE_EQ(P[1], out[1]);
    EXPECT_DOUBLE_EQ(P[2], out[2]);
    EXPECT_DOUBLE_EQ(0.0, extr.position()[0]);
    EXPECT_DOUBLE_EQ(0.0, extr.position()[1]);
    EXPECT_DOUBLE_EQ(0.0, extr.position()[2]);
}

TEST(CameraExtrinsicsTest, RelativeRotationFrom_MatchesHandComputedRbTRa) {
            const double sa = 0.5, ca = std::sqrt(3.0) / 2.0;
    const double sb = -0.3, cb = std::sqrt(0.91);       const cv::Matx33d Ra(ca, 0, sa,
                         0, 1, 0,
                         -sa, 0, ca);
    const cv::Matx33d Rb(cb, sb, 0,
                        -sb, cb, 0,
                         0, 0, 1);
    const cv::Vec3d ta(1.0, 0.0, 0.0);
    const cv::Vec3d tb(2.0, 1.0, 0.5);
    CameraExtrinsics a{Ra, ta};
    CameraExtrinsics b{Rb, tb};

    const cv::Matx33d expected = Rb.t() * Ra;
    const cv::Matx33d got = b.RelativeRotationFrom(a);
    for (int r = 0; r < 3; ++r)
        for (int c2 = 0; c2 < 3; ++c2)
            EXPECT_DOUBLE_EQ(expected(r, c2), got(r, c2));
}

TEST(CameraExtrinsicsTest, ToCamera_OfOtherPosition_MatchesHandComputedTAb) {
            const double sa = 0.5, ca = std::sqrt(3.0) / 2.0;
    const double sb = -0.3, cb = std::sqrt(0.91);
    const cv::Matx33d Ra(ca, 0, sa,
                         0, 1, 0,
                         -sa, 0, ca);
    const cv::Matx33d Rb(cb, sb, 0,
                        -sb, cb, 0,
                         0, 0, 1);
    const cv::Vec3d ta(1.0, 0.0, 0.0);
    const cv::Vec3d tb(2.0, 1.0, 0.5);
    CameraExtrinsics a{Ra, ta};
    CameraExtrinsics b{Rb, tb};

    const cv::Vec3d expected = Rb.t() * (ta - tb);
    const cv::Vec3d got = b.ToCamera(a.position());
    EXPECT_DOUBLE_EQ(expected[0], got[0]);
    EXPECT_DOUBLE_EQ(expected[1], got[1]);
    EXPECT_DOUBLE_EQ(expected[2], got[2]);
}
