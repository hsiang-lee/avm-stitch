// Tests for the CameraModel abstraction + factory vocabulary (spec D1,
// plan batch A / Tasks A1+A2). Pure math: vocabulary ("kb" | "table" |
// "ocam"), error semantics (unknown model / variant mismatch / invalid
// params), and KbCameraModel numerical equivalence with the legacy
// ProjectWorldToFishEye (projection.hpp).

#include <gtest/gtest.h>

#include "geometry/camera_model.hpp"
#include "geometry/kb_camera_model.hpp"
#include "geometry/projection.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

using namespace avm_stitch;

namespace {

KbParams MakeKbParams(int width = 640, int height = 480) {
    KbParams p;
    p.intr = CameraIntrinsics{width, height, 300.0, 300.0, 320.0, 240.0,
                              0, 0, 0, 0};
    return p;
}

} // namespace

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

// ---------------------------------------------------------------------------
// Task A2: KbCameraModel numerical equivalence with ProjectWorldToFishEye.
// The migration must be pixel-identical (kb 回归, spec D4-3).
// ---------------------------------------------------------------------------

namespace {

CameraExtrinsics MakeIdentityExtrinsics() {
    return {cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)};
}

CameraIntrinsics MakeKbIntrinsics(double k1 = 0, double k2 = 0,
                                  double k3 = 0, double k4 = 0,
                                  double fx = 300.0, double fy = 300.0) {
    return {640, 480, fx, fy, 320.0, 240.0, k1, k2, k3, k4};
}

// Points that project strictly inside the principal-point pixel [0,width) x
// [0,height) are valid; anything else invalidates (spec D1 contract).
uint8_t ExpectedValid(double u, double v, int width, int height) {
    return (u >= 0.0 && u < static_cast<double>(width) &&
            v >= 0.0 && v < static_cast<double>(height)) ? 1 : 0;
}

} // namespace

TEST(KbModel, OnAxis_MatchesLegacy) {
    auto model = MakeCameraModel("kb", KbParams{MakeKbIntrinsics()});
    std::vector<cv::Vec3d> pts{cv::Vec3d(0, 0, 1)};
    std::vector<cv::Vec2d> uv(pts.size());
    std::vector<uint8_t> valid(pts.size());

    model->ProjectPoints(pts, uv, valid);
    EXPECT_DOUBLE_EQ(320.0, uv[0][0]);
    EXPECT_DOUBLE_EQ(240.0, uv[0][1]);
    EXPECT_EQ(1, valid[0]);
}

TEST(KbModel, OffAxisZeroK_MatchesLegacy) {
    auto model = MakeCameraModel("kb", KbParams{MakeKbIntrinsics()});
    std::vector<cv::Vec3d> pts{cv::Vec3d(1, 0, 1)};
    std::vector<cv::Vec2d> uv(pts.size());
    std::vector<uint8_t> valid(pts.size());

    model->ProjectPoints(pts, uv, valid);
    // Same pinned value as test_projection.cpp OffCenter: u = 320 + 300*atan(1).
    EXPECT_NEAR(320.0 + 300.0 * std::atan(1.0), uv[0][0], 1e-9);
    EXPECT_DOUBLE_EQ(240.0, uv[0][1]);
    EXPECT_EQ(1, valid[0]);
}

TEST(KbModel, WithDistortion_MatchesLegacy) {
    const CameraIntrinsics intr = MakeKbIntrinsics(0.1, -0.05, 0.01, 0.002);
    auto model = MakeCameraModel("kb", KbParams{intr});
    const CameraExtrinsics extr = MakeIdentityExtrinsics();

    // 10 known directions incl. z<0 and theta ~ 120 deg (behind hemisphere).
    const std::vector<cv::Vec3d> pts = {
        {0, 0, 1},    {1, 0, 1},    {0, 1, 1},    {0.5, 0.5, 1},
        {1, -1, 0.5}, {-2, 1, 2},   {0.3, 0.2, -0.9327}, {1, 1, -1},
        {2, -1, -0.5}, {0.1, 0.05, 0.05},
    };
    std::vector<cv::Vec2d> uv(pts.size());
    std::vector<uint8_t> valid(pts.size());

    model->ProjectPoints(pts, uv, valid);
    for (size_t i = 0; i < pts.size(); ++i) {
        double u, v;
        ProjectWorldToFishEye(pts[i], intr, extr, u, v);
        EXPECT_NEAR(u, uv[i][0], 1e-9) << "point " << i << " u mismatch";
        EXPECT_NEAR(v, uv[i][1], 1e-9) << "point " << i << " v mismatch";
    }
}

TEST(KbModel, RXYBelowEps_SnapsToPrincipalPoint) {
    auto model = MakeCameraModel("kb", KbParams{MakeKbIntrinsics()});
    std::vector<cv::Vec3d> pts{cv::Vec3d(0, 0, -1)};   // r_xy < eps, z < 0
    std::vector<cv::Vec2d> uv(pts.size());
    std::vector<uint8_t> valid(pts.size());

    model->ProjectPoints(pts, uv, valid);
    EXPECT_DOUBLE_EQ(320.0, uv[0][0]);
    EXPECT_DOUBLE_EQ(240.0, uv[0][1]);
    EXPECT_EQ(1, valid[0]);
}

TEST(KbModel, OutsideImageRectangle_Invalid) {
    // Large focal + steep angle projects outside [0,640) x [0,480).
    const CameraIntrinsics intr = MakeKbIntrinsics(0, 0, 0, 0, 3000.0, 3000.0);
    auto model = MakeCameraModel("kb", KbParams{intr});
    std::vector<cv::Vec3d> pts{cv::Vec3d(1, 0, 1)};
    std::vector<cv::Vec2d> uv(pts.size());
    std::vector<uint8_t> valid(pts.size());

    model->ProjectPoints(pts, uv, valid);   // must NOT throw (spec §5)
    EXPECT_EQ(0, valid[0]);
    // uv still written even when invalid (caller may inspect it).
    EXPECT_TRUE(uv[0][0] >= 640.0 || uv[0][0] < 0.0);
}

TEST(KbModel, BatchMatchesSingleCalls) {
    const CameraIntrinsics intr = MakeKbIntrinsics(0.1, -0.05, 0.01, 0.002);
    auto model = MakeCameraModel("kb", KbParams{intr});
    const CameraExtrinsics extr = MakeIdentityExtrinsics();

    // 100 deterministic points spread over the full sphere.
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
        double u, v;
        ProjectWorldToFishEye(pts[i], intr, extr, u, v);
        EXPECT_NEAR(u, uv_batch[i][0], 1e-12) << "point " << i;
        EXPECT_NEAR(v, uv_batch[i][1], 1e-12) << "point " << i;
        const bool in_rect = ExpectedValid(uv_batch[i][0], uv_batch[i][1],
                                           intr.width, intr.height) != 0;
        EXPECT_EQ(in_rect ? 1 : 0, valid_batch[i]) << "point " << i;
    }

    // Mismatched span lengths -> std::invalid_argument.
    std::vector<cv::Vec2d> short_uv(pts.size() - 1);
    std::vector<uint8_t> short_valid(pts.size());
    EXPECT_THROW(model->ProjectPoints(pts, short_uv, short_valid),
                 std::invalid_argument);
}
