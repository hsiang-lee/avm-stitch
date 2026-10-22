// truth chain lives in test_projection.cpp).

#include <gtest/gtest.h>

#include "camera_model/camera.hpp"
#include "camera_model/kb_camera_model.hpp"
#include "camera_model/camera_extrinsics.hpp"

#include <bit>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

using namespace avm_stitch;

namespace {

KbParams MakeKbParams(int width = 640, int height = 480) {
    KbParams p;
    p.width = width;
    p.height = height;
    p.fx = 300.0;
    p.fy = 300.0;
    p.cx = 320.0;
    p.cy = 240.0;
    return p;
}

}
TEST(MakeCameraModel, Kb_ReturnsModel) {
    auto model = MakeCameraModel("kb", MakeKbParams());
    ASSERT_NE(model, nullptr);
    EXPECT_EQ(640, model->width());
    EXPECT_EQ(480, model->height());
}

TEST(MakeCameraModel, UnknownModel_Throws) {
    try {
        auto model = MakeCameraModel("fov", MakeKbParams());
        (void)model;
        FAIL() << "expected std::invalid_argument for unknown model";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find("fov"), std::string::npos)
            << "what() should name the unknown model, got: " << e.what();
    }
}

TEST(MakeCameraModel, VariantMismatch_Throws) {
    try {
        auto model = MakeCameraModel("kb", LutParams{});
        (void)model;
        FAIL() << "expected std::invalid_argument for variant mismatch";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find("kb"), std::string::npos)
            << "what() should name the requested model, got: " << e.what();
    }
}

TEST(MakeCameraModel, NonPositiveSize_Throws) {
    try {
        auto model = MakeCameraModel("kb", MakeKbParams(/*width=*/0, /*height=*/480));
        (void)model;
        FAIL() << "expected std::invalid_argument for non-positive size";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find("width"), std::string::npos)
            << "what() should mention width, got: " << e.what();
    }
}

// point to the ProjectCameraPoint override with the same camera-frame
// polynomial's cv::fisheye oracle lives in test_projection.cpp
// (KbCameraModel::ProjectCameraPoint vs cv::fisheye::projectPoints).

namespace {

CameraExtrinsics MakeIdentityExtrinsics() {
    return {cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)};
}

KbParams MakeKbIntrinsics(double k1 = 0, double k2 = 0,
                          double k3 = 0, double k4 = 0,
                          double fx = 300.0, double fy = 300.0) {
    KbParams p;
    p.width = 640;
    p.height = 480;
    p.fx = fx;
    p.fy = fy;
    p.cx = 320.0;
    p.cy = 240.0;
    p.k1 = k1;
    p.k2 = k2;
    p.k3 = k3;
    p.k4 = k4;
    return p;
}

uint8_t ExpectedValid(double u, double v, int width, int height) {
    return (u >= 0.0 && u < static_cast<double>(width) &&
            v >= 0.0 && v < static_cast<double>(height)) ? 1 : 0;
}

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
        {0, 0, 1},    {1, 0, 1},    {0, 1, 1},    {0.5, 0.5, 1},
        {1, -1, 0.5}, {-2, 1, 2},   {0.3, 0.2, -0.9327}, {1, 1, -1},
        {2, -1, -0.5}, {0.1, 0.05, 0.05},
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
    std::vector<cv::Vec3d> pts{cv::Vec3d(0, 0, -1)};       std::vector<cv::Vec2d> uv(pts.size());
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

    model->ProjectPoints(pts, uv, valid);       EXPECT_EQ(0, valid[0]);
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


TEST(Camera, DefaultIsInvalid) {
    Camera cam;
    EXPECT_FALSE(cam.Valid());
}

TEST(Camera, ConstructedIsValidAndDelegates) {
    Camera cam(MakeIdentityExtrinsics(),
               std::make_unique<KbCameraModel>(MakeKbIntrinsics()));
    EXPECT_TRUE(cam.Valid());
    EXPECT_EQ(640, cam.Width());
    EXPECT_EQ(480, cam.Height());
    EXPECT_EQ(320.0, cam.Cx());
    EXPECT_EQ(240.0, cam.Cy());
}

TEST(Camera, MovePreservesValidity) {
    Camera src(MakeIdentityExtrinsics(),
               std::make_unique<KbCameraModel>(MakeKbIntrinsics()));
    ASSERT_TRUE(src.Valid());
    Camera dst = std::move(src);
    EXPECT_TRUE(dst.Valid());
    EXPECT_FALSE(src.Valid());
}

// inscribed circle). The formulas were moved verbatim from the retired
// hand on the same inputs and must reproduce the exact same double bits.

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
        if (feather_px <= 0.0) return 1.0;
        const double r_px = std::hypot(u - cx, v - cy);
        const double margin = radius - r_px;
        if (margin >= feather_px) return 1.0;
        return 0.0;
    };

    const auto bit_u = [](double x) { return std::bit_cast<uint64_t>(x); };
    const std::vector<cv::Vec2d> samples = {
        {320.0, 240.0}, {320.0, 340.0}, {420.0, 300.0},              {560.0, 400.0},                                       // r~288, margin~12 < 40 -> gate 0.0
        {620.0, 460.0},                                       // rect in, circle out (r~372 > 300)
        {320.0, 240.0 + 190.0},                                   };
    for (const cv::Vec2d& uv : samples) {
        SCOPED_TRACE("sample (" + std::to_string(uv[0]) + ", "
                     + std::to_string(uv[1]) + ")");
        const bool circle_expected = inside(uv[0], uv[1]);
        EXPECT_EQ(circle_expected, model->InsideFisheyeCircle(uv[0], uv[1]));
        EXPECT_EQ(circle_expected, cam.InsideFisheyeCircle(uv[0], uv[1]));
        const uint64_t feather_expected = bit_u(feather(uv[0], uv[1]));
        EXPECT_EQ(feather_expected, bit_u(model->EdgeFeather(uv[0], uv[1])));
        EXPECT_EQ(feather_expected, bit_u(cam.EdgeFeather(uv[0], uv[1])));
    }
}
