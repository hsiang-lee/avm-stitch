// Tests for OcamCameraModel (plan batch B / Task B3).
//
// Sign conventions (OQ-2, locked): the avm-stitch camera frame is OpenCV
// (+Z into the scene) while the OCAM front hemisphere is Z<0, so the
// implementation consumes the flipped ray (x, y, -z) internally. The optical
// axis (0,0,1) maps to the principal point.
//
// Reference values are pinned from the renderer kernel
// avm_mapper::OcamModel::world2cam(x, y, -z, ...) (header-only, probed once
// at RED with the exact coefficients below and written down here, so the
// committed test does not depend on the vcpkg tree):
//   direction (0.3, 0.2, 0.9327) -> (742.00773299911305, 546.37971005202451)

#include <gtest/gtest.h>

#include "geometry/camera_model.hpp"
#include "geometry/ocam_camera_model.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

using namespace avm_stitch;

namespace {

// fish_back from avm-worker/tests/fixtures/intrinsics/intrinsics_three_models.xml
// (lines 11-15), pinned verbatim (plan B3 constants).
OcamParams MakeFishBackParams() {
    OcamParams p;
    p.pol = {-319.5284, 0.0, 8.991405e-04, -7.701385e-07, 1.365101e-09};
    p.invpol = {589.4893,   428.471,   -6.866877,  8.354607,  75.49948,
                -5.837637, -54.56404,  24.77632,  70.42449,  37.77541,
                6.678202};
    p.cx = 643.0012;
    p.cy = 480.3732;
    p.c = 0.9999853;
    p.d = 1.297014e-05;
    p.e = 1.772778e-05;
    p.width = 1280;
    p.height = 960;
    return p;
}

} // namespace

TEST(OcamModel, AxisPoint_ProjectsToPrincipalPoint) {
    auto model = MakeCameraModel("ocam", MakeFishBackParams());
    std::vector<cv::Vec3d> pts{cv::Vec3d(0, 0, 1)};
    std::vector<cv::Vec2d> uv(pts.size());
    std::vector<uint8_t> valid(pts.size());

    model->ProjectPoints(pts, uv, valid);
    EXPECT_DOUBLE_EQ(643.0012, uv[0][0]);
    EXPECT_DOUBLE_EQ(480.3732, uv[0][1]);
    EXPECT_EQ(1, valid[0]);
}

TEST(OcamModel, ReferencePoint_MatchesRendererKernel) {
    auto model = MakeCameraModel("ocam", MakeFishBackParams());
    // Normalized non-axis direction (plan B3 example).
    const double n = std::sqrt(0.3 * 0.3 + 0.2 * 0.2 + 0.9327 * 0.9327);
    std::vector<cv::Vec3d> pts{cv::Vec3d(0.3 / n, 0.2 / n, 0.9327 / n)};
    std::vector<cv::Vec2d> uv(pts.size());
    std::vector<uint8_t> valid(pts.size());

    model->ProjectPoints(pts, uv, valid);
    // Renderer pin (see file header): world2cam(0.3, 0.2, -0.9327).
    EXPECT_NEAR(742.00773299911305, uv[0][0], 1e-6);
    EXPECT_NEAR(546.37971005202451, uv[0][1], 1e-6);
    EXPECT_EQ(1, valid[0]);
}

TEST(OcamModel, RoundTrip_ClosesUnder01Px) {
    auto model = MakeCameraModel("ocam", MakeFishBackParams());
    const OcamParams params = MakeFishBackParams();

    // 10 deterministic pixel points across the image.
    const std::vector<cv::Vec2d> pixels = {
        {643.0, 480.4}, {700.0, 500.0}, {580.0, 430.0}, {900.0, 700.0},
        {400.0, 200.0}, {1100.0, 850.0}, {300.0, 600.0}, {750.0, 300.0},
        {650.0, 900.0}, {1000.0, 100.0},
    };
    for (const cv::Vec2d& px : pixels) {
        // Forward model pixel -> ray (same polynomial the model inverts).
        const double invdet = 1.0 / (params.c - params.d * params.e);
        const double mx = px[0] - params.cx;
        const double my = px[1] - params.cy;
        const double xp = invdet * (mx - params.d * my);
        const double yp = invdet * (-params.e * mx + params.c * my);
        double zp = params.pol[0];   // a0 dominates the cam2world polynomial
        double r_i = 1.0;
        for (size_t i = 1; i < params.pol.size(); ++i) {
            r_i *= std::hypot(xp, yp);
            zp += r_i * params.pol[i];
        }
        // avm-stitch frame: +Z into the scene -> ray (xp, yp, -zp).
        const std::vector<cv::Vec3d> pts{cv::Vec3d(xp, yp, -zp)};
        std::vector<cv::Vec2d> uv(pts.size());
        std::vector<uint8_t> valid(pts.size());
        model->ProjectPoints(pts, uv, valid);

        const double du = std::abs(uv[0][0] - px[0]);
        const double dv = std::abs(uv[0][1] - px[1]);
        EXPECT_LT(du, 0.01) << "pixel (" << px[0] << ", " << px[1] << ") u";
        EXPECT_LT(dv, 0.01) << "pixel (" << px[0] << ", " << px[1] << ") v";
    }
}

TEST(OcamModel, BehindCamera_Invalid) {
    auto model = MakeCameraModel("ocam", MakeFishBackParams());
    std::vector<cv::Vec3d> pts{cv::Vec3d(0.5, 0.0, -0.866)};   // z < 0
    std::vector<cv::Vec2d> uv(pts.size());
    std::vector<uint8_t> valid(pts.size());

    model->ProjectPoints(pts, uv, valid);   // must NOT throw (spec §5)
    EXPECT_EQ(0, valid[0]);
}

TEST(OcamModel, EmptyInvpol_Throws) {
    OcamParams p = MakeFishBackParams();
    p.invpol.clear();
    EXPECT_THROW(MakeCameraModel("ocam", p), std::invalid_argument);
}

TEST(OcamModel, NonPositiveSize_Throws) {
    OcamParams p = MakeFishBackParams();
    p.width = 0;
    try {
        auto model = MakeCameraModel("ocam", p);
        (void)model;
        FAIL() << "expected std::invalid_argument for width=0";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find("width"), std::string::npos)
            << "what() should mention width, got: " << e.what();
    }
}

TEST(OcamModel, BatchMatchesSingle) {
    auto model = MakeCameraModel("ocam", MakeFishBackParams());

    // 30 deterministic directions spread over the front hemisphere.
    std::vector<cv::Vec3d> pts;
    for (int i = 0; i < 30; ++i) {
        const double t = i * 0.63;
        pts.push_back(cv::Vec3d(std::sin(t), std::cos(t * 0.7),
                                std::abs(std::cos(t)) + 1e-6));
    }
    std::vector<cv::Vec2d> uv_batch(pts.size());
    std::vector<uint8_t> valid_batch(pts.size());
    model->ProjectPoints(pts, uv_batch, valid_batch);

    for (size_t i = 0; i < pts.size(); ++i) {
        std::vector<cv::Vec3d> one{pts[i]};
        std::vector<cv::Vec2d> uv_one(one.size());
        std::vector<uint8_t> valid_one(one.size());
        model->ProjectPoints(one, uv_one, valid_one);
        // Same implementation, same code path: bit-exact (tolerance 0).
        EXPECT_EQ(uv_one[0][0], uv_batch[i][0]) << "point " << i << " u";
        EXPECT_EQ(uv_one[0][1], uv_batch[i][1]) << "point " << i << " v";
        EXPECT_EQ(valid_one[0], valid_batch[i]) << "point " << i << " valid";
    }

    // Mismatched span lengths -> std::invalid_argument.
    std::vector<cv::Vec2d> short_uv(pts.size() - 1);
    std::vector<uint8_t> short_valid(pts.size());
    EXPECT_THROW(model->ProjectPoints(pts, short_uv, short_valid),
                 std::invalid_argument);
}
