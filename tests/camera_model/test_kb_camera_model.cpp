
#include <gtest/gtest.h>

#include <opencv2/calib3d.hpp>

#include "camera_model/camera.hpp"
#include "camera_model/camera_extrinsics.hpp"
#include "camera_model/camera_model.hpp"
#include "camera_model/kb_camera_model.hpp"

#include "test_support/geometry_test_helpers.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

using namespace avm_stitch;

namespace {

constexpr double kPi = 3.14159265358979323846;

using avm_stitch::test::MakeIdentityExtrinsics;
using avm_stitch::test::MakeKbIntrinsics;

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
    if (cv::norm(r) < 1e-9)
        r = cv::Vec3d(1.0, 0.0, 0.0);
    r /= cv::norm(r);
    cv::Vec3d u = f.cross(r);
    return cv::Matx33d(r[0], u[0], f[0],
                       r[1], u[1], f[1],
                       r[2], u[2], f[2]);
}

uint8_t ExpectedValid(double u, double v, int width, int height) {
    return (u >= 0.0 && u < static_cast<double>(width) &&
            v >= 0.0 && v < static_cast<double>(height))
               ? 1
               : 0;
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
    EXPECT_NEAR(320.0 + 300.0 * std::atan(1.0), uv[0], 1e-6);
    EXPECT_NEAR(240.0, uv[1], 1e-6);
}

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
        {0, 0, 0.2}, {1.0, 0.0, 0.2}, {0.0, 1.0, 0.2}, {2.0, 2.0, 0.5}, {-2.0, -1.0, 0.8}, {3.0, 0.0, 0.6}};

    cv::Matx33d Rcw = extr.RelativeRotationFrom(avm_stitch::CameraExtrinsics{});
    cv::Vec3d tcw = -Rcw * extr.position();
    cv::Mat rvec;
    cv::Rodrigues(cv::Mat(Rcw), rvec);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << tcw[0], tcw[1], tcw[2]);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 441.45, 0, 952.14, 0, 441.45, 758.27, 0, 0, 1);
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

TEST(ProjectionTest, InsideFisheyeCircle_OnBoundary) {
    KbCameraModel intr = MakeKbModel(640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0);
    EXPECT_TRUE(intr.InsideFisheyeCircle(320.0 + 240.0, 240.0));
    EXPECT_FALSE(intr.InsideFisheyeCircle(320.0 + 240.0 + 1e-6, 240.0));
}

TEST(ProjectionTest, EdgeFeather_NearEdge_HardGate_Zero) {
    KbCameraModel intr = MakeKbModel(640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0);
    EXPECT_DOUBLE_EQ(0.0, intr.EdgeFeather(320.0, 240.0 + 230.0));
}

TEST(ProjectionTest, EdgeFeather_ZeroFeatherWidth) {
    KbCameraModel intr = MakeKbModel(640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0);
    EXPECT_NEAR(1.0, intr.EdgeFeather(320.0, 240.0 + 230.0, 0.0), 1e-12);
    EXPECT_NEAR(1.0, intr.EdgeFeather(320.0 + 239.0, 240.0, -5.0), 1e-12);
}

TEST(KbModel, OnAxis_MatchesLegacy) {
    auto model = MakeCameraModel("kb", MakeKbIntrinsics());
    std::vector<cv::Vec3d> pts{cv::Vec3d(0, 0, 1)};
    std::vector<cv::Vec2d> uv(pts.size());
    std::vector<uint8_t> valid(pts.size());

    model->ProjectPoints(pts, uv, valid);
    EXPECT_DOUBLE_EQ(320.0, uv[0][0]);
    EXPECT_DOUBLE_EQ(240.0, uv[0][1]);
    EXPECT_EQ(1, valid[0]);
}

TEST(KbModel, OffAxisZeroK_MatchesLegacy) {
    auto model = MakeCameraModel("kb", MakeKbIntrinsics());
    std::vector<cv::Vec3d> pts{cv::Vec3d(1, 0, 1)};
    std::vector<cv::Vec2d> uv(pts.size());
    std::vector<uint8_t> valid(pts.size());

    model->ProjectPoints(pts, uv, valid);
    EXPECT_NEAR(320.0 + 300.0 * std::atan(1.0), uv[0][0], 1e-9);
    EXPECT_DOUBLE_EQ(240.0, uv[0][1]);
    EXPECT_EQ(1, valid[0]);
}

TEST(KbModel, WithDistortion_MatchesLegacy) {
    const KbParams intr = MakeKbIntrinsics(0.1, -0.05, 0.01, 0.002);
    auto model = MakeCameraModel("kb", intr);
    const CameraExtrinsics extr = MakeIdentityExtrinsics();

    const std::vector<cv::Vec3d> pts = {
        {0, 0, 1},
        {1, 0, 1},
        {0, 1, 1},
        {0.5, 0.5, 1},
        {1, -1, 0.5},
        {-2, 1, 2},
        {0.3, 0.2, -0.9327},
        {1, 1, -1},
        {2, -1, -0.5},
        {0.1, 0.05, 0.05},
    };
    std::vector<cv::Vec2d> uv(pts.size());
    std::vector<uint8_t> valid(pts.size());

    model->ProjectPoints(pts, uv, valid);
    for (size_t i = 0; i < pts.size(); ++i) {
        // override) on the same camera-frame transform.
        const cv::Vec2d ref = model->ProjectCameraPoint(extr.ToCamera(pts[i]));
        EXPECT_NEAR(ref[0], uv[i][0], 1e-9) << "point " << i << " u mismatch";
        EXPECT_NEAR(ref[1], uv[i][1], 1e-9) << "point " << i << " v mismatch";
    }
}

TEST(KbModel, RXYBelowEps_SnapsToPrincipalPoint) {
    auto model = MakeCameraModel("kb", MakeKbIntrinsics());
    std::vector<cv::Vec3d> pts{cv::Vec3d(0, 0, -1)};
    std::vector<cv::Vec2d> uv(pts.size());
    std::vector<uint8_t> valid(pts.size());

    model->ProjectPoints(pts, uv, valid);
    EXPECT_DOUBLE_EQ(320.0, uv[0][0]);
    EXPECT_DOUBLE_EQ(240.0, uv[0][1]);
    EXPECT_EQ(1, valid[0]);
}

TEST(KbModel, OutsideImageRectangle_Invalid) {
    const KbParams intr = MakeKbIntrinsics(0, 0, 0, 0, 3000.0, 3000.0);
    auto model = MakeCameraModel("kb", intr);
    std::vector<cv::Vec3d> pts{cv::Vec3d(1, 0, 1)};
    std::vector<cv::Vec2d> uv(pts.size());
    std::vector<uint8_t> valid(pts.size());

    model->ProjectPoints(pts, uv, valid);
    EXPECT_EQ(0, valid[0]);
    EXPECT_TRUE(uv[0][0] >= 640.0 || uv[0][0] < 0.0);
}

TEST(KbModel, BatchMatchesSingleCalls) {
    const KbParams intr = MakeKbIntrinsics(0.1, -0.05, 0.01, 0.002);
    auto model = MakeCameraModel("kb", intr);
    const CameraExtrinsics extr = MakeIdentityExtrinsics();

    std::vector<cv::Vec3d> pts;
    for (int i = 0; i < 100; ++i) {
        const double t = i * 0.63;
        pts.push_back(cv::Vec3d(std::sin(t), std::cos(t * 0.7),
                                std::cos(t) * (i % 2 == 0 ? 1.0 : -1.0)));
    }
    std::vector<cv::Vec2d> uv_batch(pts.size());
    std::vector<uint8_t> valid_batch(pts.size());
    model->ProjectPoints(pts, uv_batch, valid_batch);

    for (size_t i = 0; i < pts.size(); ++i) {
        const cv::Vec2d ref = model->ProjectCameraPoint(extr.ToCamera(pts[i]));
        EXPECT_NEAR(ref[0], uv_batch[i][0], 1e-12) << "point " << i;
        EXPECT_NEAR(ref[1], uv_batch[i][1], 1e-12) << "point " << i;
        const bool in_rect = ExpectedValid(uv_batch[i][0], uv_batch[i][1],
                                           intr.width, intr.height) != 0;
        EXPECT_EQ(in_rect ? 1 : 0, valid_batch[i]) << "point " << i;
    }

    std::vector<cv::Vec2d> short_uv(pts.size() - 1);
    std::vector<uint8_t> short_valid(pts.size());
    EXPECT_THROW(model->ProjectPoints(pts, short_uv, short_valid),
                 std::invalid_argument);
}

TEST(CameraModelGating, KbExplicitRadius_ForwardedToFisheyeCircle) {
    KbParams p = MakeKbIntrinsics(0, 0, 0, 0);
    p.fisheye_radius = 300.0;
    auto model = MakeCameraModel("kb", p);
    ASSERT_NE(model, nullptr);
    EXPECT_NEAR(300.0, model->FisheyeCircleRadius(), 1e-9);
}

TEST(CameraModelGating, KbZeroRadius_AutoInscribedCircle) {
    auto model = MakeCameraModel("kb", MakeKbIntrinsics());
    ASSERT_NE(model, nullptr);
    EXPECT_NEAR(240.0, model->FisheyeCircleRadius(), 1e-9);
}

TEST(CameraModelGating, Model_MatchesHandFormulaBitExact) {
    KbParams p = MakeKbIntrinsics(0, 0, 0, 0);
    p.fisheye_radius = 300.0;
    auto model = MakeCameraModel("kb", p);
    ASSERT_NE(model, nullptr);
    Camera cam(MakeIdentityExtrinsics(), std::make_unique<KbCameraModel>(p));
    ASSERT_TRUE(cam.Valid());

    // Hand-written replica of the moved base gating formulas (this is the
    // pre-migration hand-written intrinsic body evaluated in the test itself).
    const double cx = 320.0, cy = 240.0;
    const double radius = 300.0 > 0.0 ? 300.0
                                      : std::max(0.0, std::min({cx, cy, 640.0 - cx, 480.0 - cy}));
    const auto inside = [&](double u, double v) {
        return std::hypot(u - cx, v - cy) <= radius;
    };
    const auto feather = [&](double u, double v, double feather_px = 40.0) {
        if (feather_px <= 0.0)
            return 1.0;
        const double r_px = std::hypot(u - cx, v - cy);
        const double margin = radius - r_px;
        if (margin >= feather_px)
            return 1.0;
        return 0.0;
    };

    const auto bit_u = [](double x) { return std::bit_cast<uint64_t>(x); };
    const std::vector<cv::Vec2d> samples = {
        {320.0, 240.0},
        {320.0, 340.0},
        {420.0, 300.0},
        {560.0, 400.0}, // r~288, margin~12 < 40 -> gate 0.0
        {620.0, 460.0}, // rect in, circle out (r~372 > 300)
        {320.0, 240.0 + 190.0},
    };
    for (const cv::Vec2d& uv : samples) {
        SCOPED_TRACE("sample (" + std::to_string(uv[0]) + ", " + std::to_string(uv[1]) + ")");
        const bool circle_expected = inside(uv[0], uv[1]);
        EXPECT_EQ(circle_expected, model->InsideFisheyeCircle(uv[0], uv[1]));
        EXPECT_EQ(circle_expected, cam.InsideFisheyeCircle(uv[0], uv[1]));
        const uint64_t feather_expected = bit_u(feather(uv[0], uv[1]));
        EXPECT_EQ(feather_expected, bit_u(model->EdgeFeather(uv[0], uv[1])));
        EXPECT_EQ(feather_expected, bit_u(cam.EdgeFeather(uv[0], uv[1])));
    }
}
