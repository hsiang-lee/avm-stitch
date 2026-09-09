// Tests for avm_stitch projection (3D world -> fisheye pixel chain).
// Pure math, no GL / OSG dependency. Reference oracle: cv::fisheye::projectPoints.

#include <gtest/gtest.h>

#include <opencv2/calib3d.hpp>

#include "geometry/projection.hpp"

#include <cmath>
#include <vector>

using namespace avm_stitch;

namespace {

constexpr double kPi = 3.14159265358979323846;

// Build a camera-to-world rotation R_wc such that the camera at `pos` looks at
// `target` along its local +Z axis. Columns of R_wc are the camera axes in
// world space: [right | down | forward].
cv::Matx33d MakeLookAtRotation(const cv::Vec3d& pos, const cv::Vec3d& target) {
    cv::Vec3d f = target - pos;
    f /= cv::norm(f);
    cv::Vec3d world_up(0.0, 0.0, 1.0);
    cv::Vec3d r = world_up.cross(f);
    if (cv::norm(r) < 1e-9) r = cv::Vec3d(1.0, 0.0, 0.0);
    r /= cv::norm(r);
    cv::Vec3d u = -f.cross(r);
    return cv::Matx33d(r[0], u[0], f[0],
                       r[1], u[1], f[1],
                       r[2], u[2], f[2]);
}

} // namespace

// ---------------------------------------------------------------------------
// ProjectWorldToFishEye
// ---------------------------------------------------------------------------

TEST(ProjectionTest, IdentityCamera_Center) {
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)};
    double u, v;
    ProjectWorldToFishEye(cv::Vec3d(0, 0, 1), intr, extr, u, v);
    EXPECT_NEAR(320.0, u, 1e-6);
    EXPECT_NEAR(240.0, v, 1e-6);
}

TEST(ProjectionTest, IdentityCamera_OffCenter) {
    // Fisheye with zero distortion: u = cx + fx * atan(x_n), NOT cx + fx*x_n.
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)};
    double u, v;
    ProjectWorldToFishEye(cv::Vec3d(1, 0, 1), intr, extr, u, v);
    EXPECT_NEAR(320.0 + 300.0 * std::atan(1.0), u, 1e-6);   // x_n = 1, theta = pi/4
    EXPECT_NEAR(240.0, v, 1e-6);                            // y_n = 0
}

TEST(ProjectionTest, WithRotation) {
    // R_wc = Ry(30deg), t = 0. P_cam = Ry(30)^T * (0,0,1) = (-sin30, 0, cos30).
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    double s = 0.5, c = std::sqrt(3.0) / 2.0;
    CameraExtrinsics extr{cv::Matx33d(c, 0, s,
                                      0, 1, 0,
                                     -s, 0, c),
                          cv::Vec3d(0, 0, 0)};
    double u, v;
    ProjectWorldToFishEye(cv::Vec3d(0, 0, 1), intr, extr, u, v);
    // P_cam = (-0.5, 0, 0.866): x_n = -tan(30), theta = 30deg = pi/6
    EXPECT_NEAR(320.0 - 300.0 * (kPi / 6.0), u, 1e-6);
    EXPECT_NEAR(240.0, v, 1e-6);
}

TEST(ProjectionTest, WithTranslation) {
    // P_cam = R^T (P - t) = (0,0,1) -> on-axis -> image center.
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(1, 0, 0)};
    double u, v;
    ProjectWorldToFishEye(cv::Vec3d(1, 0, 1), intr, extr, u, v);
    EXPECT_NEAR(320.0, u, 1e-6);
    EXPECT_NEAR(240.0, v, 1e-6);
}

TEST(ProjectionTest, MatchesOpenCV) {
    // Compare against cv::fisheye::projectPoints for a look-at camera with
    // distortion. NOTE: extr.R is T_wc (camera->world); cv::fisheye needs
    // world->camera, so pass R_cw = R_wc^T and t_cw = -R_cw * t_wc.
    CameraIntrinsics intr{1920, 1536, 441.45, 441.45, 952.14, 758.27,
                          0.215, -0.05, 0.003, 0.0};
    cv::Vec3d pos(3.84, -0.016, 0.66);
    cv::Vec3d target(0.0, 0.0, 0.2);
    CameraExtrinsics extr{MakeLookAtRotation(pos, target), pos};

    std::vector<cv::Vec3d> points = {
        {0, 0, 0.2}, {1.0, 0.0, 0.2}, {0.0, 1.0, 0.2},
        {2.0, 2.0, 0.5}, {-2.0, -1.0, 0.8}, {3.0, 0.0, 0.6}
    };

    // world -> camera rvec/tvec for cv::fisheye
    cv::Matx33d Rcw = extr.R.t();
    cv::Vec3d tcw = -Rcw * extr.t;
    cv::Mat rvec;
    cv::Rodrigues(cv::Mat(Rcw), rvec);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << tcw[0], tcw[1], tcw[2]);
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
        intr.fx, 0, intr.cx, 0, intr.fy, intr.cy, 0, 0, 1);
    cv::Mat D = (cv::Mat_<double>(4, 1) << intr.k1, intr.k2, intr.k3, intr.k4);

    int forward_count = 0;
    for (const auto& P : points) {
        double u, v;
        ProjectWorldToFishEye(P, intr, extr, u, v);
        if (u < 0.0 || v < 0.0) continue;   // behind camera -> no cv equivalent
        ++forward_count;

        std::vector<cv::Point3d> obj_pts(1, cv::Point3d(P[0], P[1], P[2]));
        std::vector<cv::Point2d> img_pts;
        cv::fisheye::projectPoints(obj_pts, img_pts, rvec, tvec, K, D);
        EXPECT_NEAR(img_pts[0].x, u, 0.01);
        EXPECT_NEAR(img_pts[0].y, v, 0.01);
    }
    EXPECT_GT(forward_count, 0);   // at least some points must be in front
}

TEST(ProjectionTest, PointAtCameraZ) {
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)};
    double u, v;
    ProjectWorldToFishEye(cv::Vec3d(0, 0, 2), intr, extr, u, v);
    EXPECT_NEAR(320.0, u, 1e-6);
    EXPECT_NEAR(240.0, v, 1e-6);
}

TEST(ProjectionTest, LargeDistortionK1) {
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 1.0, 0, 0, 0};
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)};
    double u, v;
    ProjectWorldToFishEye(cv::Vec3d(0.5, 0, 1), intr, extr, u, v);
    EXPECT_TRUE(std::isfinite(u));
    EXPECT_TRUE(std::isfinite(v));
    // theta_d = theta*(1 + k1*theta^2); theta = atan(0.5)
    double theta = std::atan(0.5);
    double theta_d = theta * (1.0 + 1.0 * theta * theta);
    EXPECT_NEAR(320.0 + 300.0 * theta_d, u, 1e-6);
    EXPECT_NEAR(240.0, v, 1e-6);
}

// ---------------------------------------------------------------------------
// IsForwardFacing
// ---------------------------------------------------------------------------

TEST(ProjectionTest, IsForwardFacing_TrueForPointInFront) {
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)};
    EXPECT_TRUE(IsForwardFacing(cv::Vec3d(0, 0, 1), extr));
}

TEST(ProjectionTest, IsForwardFacing_FalseForPointBehind) {
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)};
    EXPECT_FALSE(IsForwardFacing(cv::Vec3d(0, 0, -1), extr));
}

TEST(ProjectionTest, IsForwardFacing_FalseForPointAtZEqualZero) {
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)};
    EXPECT_FALSE(IsForwardFacing(cv::Vec3d(0, 0, 0), extr));
}

// ---------------------------------------------------------------------------
// FisheyeCircleRadius / IsInsideFisheyeCircle
// ---------------------------------------------------------------------------

TEST(ProjectionTest, FisheyeCircleRadius_AutoInscribedCircle) {
    // Auto radius = min(cx, cy, width-cx, height-cy) = min(320,240,320,240).
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    EXPECT_NEAR(240.0, FisheyeCircleRadius(intr), 1e-9);
}

TEST(ProjectionTest, FisheyeCircleRadius_ExplicitOverridesAuto) {
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    intr.fisheye_radius = 300.0;
    EXPECT_NEAR(300.0, FisheyeCircleRadius(intr), 1e-9);
}

TEST(ProjectionTest, FisheyeCircleRadius_ExplicitZeroFallsBackToAuto) {
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    intr.fisheye_radius = 0.0;
    EXPECT_NEAR(240.0, FisheyeCircleRadius(intr), 1e-9);
}

TEST(ProjectionTest, IsInsideFisheyeCircle_InsideRectAndCircle) {
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    // (410,330): inside the rect, r = hypot(90,90) = 127.3 < 240.
    EXPECT_TRUE(IsInsideFisheyeCircle(intr, 410.0, 330.0));
}

TEST(ProjectionTest, IsInsideFisheyeCircle_InsideRectOutsideCircle) {
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    // (500,420): inside the 640x480 rect (the black-dead-zone case) but
    // r = hypot(180,180) = 254.6 > 240 -> outside the effective circle.
    EXPECT_FALSE(IsInsideFisheyeCircle(intr, 500.0, 420.0));
}

TEST(ProjectionTest, IsInsideFisheyeCircle_OnBoundary) {
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    // Exactly on the r = 240 circle (right of centre): still valid content.
    EXPECT_TRUE(IsInsideFisheyeCircle(intr, 320.0 + 240.0, 240.0));
    EXPECT_FALSE(IsInsideFisheyeCircle(intr, 320.0 + 240.0 + 1e-6, 240.0));
}

// ---------------------------------------------------------------------------
// FisheyeEdgeFeather
// ---------------------------------------------------------------------------

TEST(ProjectionTest, FisheyeEdgeFeather_InsideFull) {
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    // r = 100 px from the principal point; radius 240 -> margin 140 >= 16.
    EXPECT_NEAR(1.0, FisheyeEdgeFeather(intr, 320.0, 340.0), 1e-12);
}

TEST(ProjectionTest, FisheyeEdgeFeather_NearEdge) {
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    // r = 230 px, margin = 10 < 16 -> feather = 10 / 16 = 0.625.
    EXPECT_NEAR(10.0 / 16.0, FisheyeEdgeFeather(intr, 320.0, 240.0 + 230.0), 1e-9);
}

TEST(ProjectionTest, FisheyeEdgeFeather_ZeroFeatherWidth) {
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    // feather_px <= 0 disables the fade entirely.
    EXPECT_NEAR(1.0, FisheyeEdgeFeather(intr, 320.0, 240.0 + 230.0, 0.0), 1e-12);
    EXPECT_NEAR(1.0, FisheyeEdgeFeather(intr, 320.0 + 239.0, 240.0, -5.0), 1e-12);
}
