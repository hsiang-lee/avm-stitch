
#include <gtest/gtest.h>

#include <opencv2/calib3d.hpp>

#include "camera_model/kb_camera_model.hpp"
#include "camera_model/camera_extrinsics.hpp"

#include <cmath>
#include <vector>

using namespace avm_stitch;

namespace {

constexpr double kPi = 3.14159265358979323846;

// Hand-written equidistant model factory (the retired hand-written intrinsic
// model folded into KbCameraModel; KbParams is flat, fisheye_radius is the
// last field, 0 = auto inscribed circle).
KbCameraModel MakeKbModel(int width, int height, double fx, double fy,
                          double cx, double cy, double k1, double k2,
                          double k3, double k4, double fisheye_radius = 0.0) {
    const KbParams p{width, height, fx, fy, cx, cy, k1, k2, k3, k4,
                     fisheye_radius};
    return KbCameraModel(p);
}

cv::Matx33d MakeLookAtRotation(const cv::Vec3d& pos, const cv::Vec3d& target) {
    cv::Vec3d f = target - pos;
    f /= cv::norm(f);
    cv::Vec3d world_up(0.0, 0.0, 1.0);
    cv::Vec3d r = world_up.cross(f);
    if (cv::norm(r) < 1e-9) r = cv::Vec3d(1.0, 0.0, 0.0);
    r /= cv::norm(r);
    cv::Vec3d u = f.cross(r);
    return cv::Matx33d(r[0], u[0], f[0],
                       r[1], u[1], f[1],
                       r[2], u[2], f[2]);
}

}

TEST(ProjectionTest, IdentityCamera_Center) {
    KbCameraModel intr = MakeKbModel(640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0);
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)};
    const cv::Vec2d uv = intr.ProjectCameraPoint(extr.ToCamera(cv::Vec3d(0, 0, 1)));
    EXPECT_NEAR(320.0, uv[0], 1e-6);
    EXPECT_NEAR(240.0, uv[1], 1e-6);
}

TEST(ProjectionTest, IdentityCamera_OffCenter) {
        KbCameraModel intr = MakeKbModel(640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0);
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)};
    const cv::Vec2d uv = intr.ProjectCameraPoint(extr.ToCamera(cv::Vec3d(1, 0, 1)));
    EXPECT_NEAR(320.0 + 300.0 * std::atan(1.0), uv[0], 1e-6);       EXPECT_NEAR(240.0, uv[1], 1e-6);                            }

TEST(ProjectionTest, WithRotation) {
        KbCameraModel intr = MakeKbModel(640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0);
    double s = 0.5, c = std::sqrt(3.0) / 2.0;
    CameraExtrinsics extr{cv::Matx33d(c, 0, s,
                                      0, 1, 0,
                                     -s, 0, c),
                          cv::Vec3d(0, 0, 0)};
    const cv::Vec2d uv = intr.ProjectCameraPoint(extr.ToCamera(cv::Vec3d(0, 0, 1)));
        EXPECT_NEAR(320.0 - 300.0 * (kPi / 6.0), uv[0], 1e-6);
    EXPECT_NEAR(240.0, uv[1], 1e-6);
}

TEST(ProjectionTest, WithTranslation) {
        KbCameraModel intr = MakeKbModel(640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0);
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(1, 0, 0)};
    const cv::Vec2d uv = intr.ProjectCameraPoint(extr.ToCamera(cv::Vec3d(1, 0, 1)));
    EXPECT_NEAR(320.0, uv[0], 1e-6);
    EXPECT_NEAR(240.0, uv[1], 1e-6);
}

TEST(ProjectionTest, MatchesOpenCV) {
                            KbCameraModel intr = MakeKbModel(1920, 1536, 441.45, 441.45, 952.14, 758.27,
                                     0.215, -0.05, 0.003, 0.0);
    cv::Vec3d pos(3.84, -0.016, 0.66);
    cv::Vec3d target(0.0, 0.0, 0.2);
    CameraExtrinsics extr{MakeLookAtRotation(pos, target), pos};

    std::vector<cv::Vec3d> points = {
        {0, 0, 0.2}, {1.0, 0.0, 0.2}, {0.0, 1.0, 0.2},
        {2.0, 2.0, 0.5}, {-2.0, -1.0, 0.8}, {3.0, 0.0, 0.6}
    };

            cv::Matx33d Rcw = extr.RelativeRotationFrom(avm_stitch::CameraExtrinsics{});
    cv::Vec3d tcw = -Rcw * extr.position();
    cv::Mat rvec;
    cv::Rodrigues(cv::Mat(Rcw), rvec);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << tcw[0], tcw[1], tcw[2]);
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
        441.45, 0, 952.14, 0, 441.45, 758.27, 0, 0, 1);
    cv::Mat D = (cv::Mat_<double>(4, 1) << 0.215, -0.05, 0.003, 0.0);

    for (const auto& P : points) {
        const cv::Vec2d uv = intr.ProjectCameraPoint(extr.ToCamera(P));

        std::vector<cv::Point3d> obj_pts(1, cv::Point3d(P[0], P[1], P[2]));
        std::vector<cv::Point2d> img_pts;
        cv::fisheye::projectPoints(obj_pts, img_pts, rvec, tvec, K, D);
        EXPECT_NEAR(img_pts[0].x, uv[0], 1e-6);
        EXPECT_NEAR(img_pts[0].y, uv[1], 1e-6);
    }
}

TEST(ProjectionTest, PointAtCameraZ) {
    KbCameraModel intr = MakeKbModel(640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0);
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)};
    const cv::Vec2d uv = intr.ProjectCameraPoint(extr.ToCamera(cv::Vec3d(0, 0, 2)));
    EXPECT_NEAR(320.0, uv[0], 1e-6);
    EXPECT_NEAR(240.0, uv[1], 1e-6);
}

TEST(ProjectionTest, LargeDistortionK1) {
    KbCameraModel intr = MakeKbModel(640, 480, 300.0, 300.0, 320.0, 240.0, 1.0, 0, 0, 0);
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)};
    const cv::Vec2d uv = intr.ProjectCameraPoint(extr.ToCamera(cv::Vec3d(0.5, 0, 1)));
    EXPECT_TRUE(std::isfinite(uv[0]));
    EXPECT_TRUE(std::isfinite(uv[1]));
        double theta = std::atan(0.5);
    double theta_d = theta * (1.0 + 1.0 * theta * theta);
    EXPECT_NEAR(320.0 + 300.0 * theta_d, uv[0], 1e-6);
    EXPECT_NEAR(240.0, uv[1], 1e-6);
}


TEST(ProjectionTest, PointBehindCamera_ReturnsValidUV) {
                KbCameraModel intr = MakeKbModel(640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0);
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)};

        {
        const cv::Vec2d uv = intr.ProjectCameraPoint(extr.ToCamera(cv::Vec3d(0, 0, -1)));
        EXPECT_NEAR(320.0, uv[0], 1e-6);
        EXPECT_NEAR(240.0, uv[1], 1e-6);
    }

        {
        const cv::Vec2d uv = intr.ProjectCameraPoint(extr.ToCamera(cv::Vec3d(1, 0, -1)));
        EXPECT_GT(uv[0], 320.0);
        EXPECT_NEAR(240.0, uv[1], 1e-6);
    }
}

TEST(ProjectionTest, DegenerateBranch_OnAxis) {
        KbCameraModel intr = MakeKbModel(640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0);
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)};
    const cv::Vec2d uv = intr.ProjectCameraPoint(extr.ToCamera(cv::Vec3d(0, 0, 1)));
    EXPECT_NEAR(320.0, uv[0], 1e-6);
    EXPECT_NEAR(240.0, uv[1], 1e-6);
}


TEST(ProjectionTest, FisheyeCircleRadius_AutoInscribedCircle) {
        KbCameraModel intr = MakeKbModel(640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0);
    EXPECT_NEAR(240.0, intr.FisheyeCircleRadius(), 1e-9);
}

TEST(ProjectionTest, FisheyeCircleRadius_ExplicitOverridesAuto) {
    KbCameraModel intr = MakeKbModel(640, 480, 300.0, 300.0, 320.0, 240.0,
                                     0, 0, 0, 0, 300.0);       EXPECT_NEAR(300.0, intr.FisheyeCircleRadius(), 1e-9);
}

TEST(ProjectionTest, FisheyeCircleRadius_ExplicitZeroFallsBackToAuto) {
    KbCameraModel intr = MakeKbModel(640, 480, 300.0, 300.0, 320.0, 240.0,
                                     0, 0, 0, 0, 0.0);       EXPECT_NEAR(240.0, intr.FisheyeCircleRadius(), 1e-9);
}

TEST(ProjectionTest, InsideFisheyeCircle_InsideRectAndCircle) {
    KbCameraModel intr = MakeKbModel(640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0);
        EXPECT_TRUE(intr.InsideFisheyeCircle(410.0, 330.0));
}

TEST(ProjectionTest, InsideFisheyeCircle_InsideRectOutsideCircle) {
    KbCameraModel intr = MakeKbModel(640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0);
            EXPECT_FALSE(intr.InsideFisheyeCircle(500.0, 420.0));
}

TEST(ProjectionTest, InsideFisheyeCircle_OnBoundary) {
    KbCameraModel intr = MakeKbModel(640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0);
        EXPECT_TRUE(intr.InsideFisheyeCircle(320.0 + 240.0, 240.0));
    EXPECT_FALSE(intr.InsideFisheyeCircle(320.0 + 240.0 + 1e-6, 240.0));
}


TEST(ProjectionTest, EdgeFeather_InsideFull) {
    KbCameraModel intr = MakeKbModel(640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0);
        EXPECT_NEAR(1.0, intr.EdgeFeather(320.0, 340.0), 1e-12);
}

TEST(ProjectionTest, EdgeFeather_NearEdge_HardGate_Zero) {
    KbCameraModel intr = MakeKbModel(640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0);
            EXPECT_DOUBLE_EQ(0.0, intr.EdgeFeather(320.0, 240.0 + 230.0));
}

TEST(ProjectionTest, EdgeFeather_Margin30_Zero) {
    KbCameraModel intr = MakeKbModel(640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0);
        EXPECT_DOUBLE_EQ(0.0, intr.EdgeFeather(320.0, 240.0 + 210.0));
}

TEST(ProjectionTest, EdgeFeather_Margin50_One) {
    KbCameraModel intr = MakeKbModel(640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0);
        EXPECT_DOUBLE_EQ(1.0, intr.EdgeFeather(320.0, 240.0 + 190.0));
}

TEST(ProjectionTest, EdgeFeather_ZeroFeatherWidth) {
    KbCameraModel intr = MakeKbModel(640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0);
        EXPECT_NEAR(1.0, intr.EdgeFeather(320.0, 240.0 + 230.0, 0.0), 1e-12);
    EXPECT_NEAR(1.0, intr.EdgeFeather(320.0 + 239.0, 240.0, -5.0), 1e-12);
}


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
