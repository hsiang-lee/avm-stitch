//
//

#include <gtest/gtest.h>

#include "camera_model/camera_model.hpp"
#include "camera_model/lut_camera_model.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

using namespace avm_stitch;

namespace {

constexpr double kTheta30HeightMm = 0.499038711;
constexpr double kTheta30Point1HeightMm = 0.500792632;
constexpr double kTheta100HeightMm = 1.949773294;
constexpr double kPixelMm = 0.003;

constexpr int kWidth = 1280;
constexpr int kHeight = 960;
constexpr double kCx = 640.0;
constexpr double kCy = 480.0;

constexpr double kConeX = 0.5;
constexpr double kConeZ = 0.86602540378443864676;
LutParams MakeLutParams() {
    LutParams p;
    p.points = {{30.000000, kTheta30HeightMm},
                {30.100000, kTheta30Point1HeightMm},
                {100.000000, kTheta100HeightMm}};
    p.pixel_size_mm = kPixelMm;
    p.width = kWidth;
    p.height = kHeight;
    p.cx = kCx;
    p.cy = kCy;
    return p;
}

}
TEST(LutModel, AxisPoint_ProjectsToPrincipalPoint) {
    auto model = MakeCameraModel("table", MakeLutParams());
    std::vector<cv::Vec3d> pts{cv::Vec3d(0, 0, 1)};
    std::vector<cv::Vec2d> uv(pts.size());
    std::vector<uint8_t> valid(pts.size());

    model->ProjectPoints(pts, uv, valid);
    EXPECT_DOUBLE_EQ(kCx, uv[0][0]);
    EXPECT_DOUBLE_EQ(kCy, uv[0][1]);
    EXPECT_EQ(1, valid[0]);
}

TEST(LutModel, Theta30_MatchesRendererKernel) {
    auto model = MakeCameraModel("table", MakeLutParams());
    std::vector<cv::Vec3d> pts{cv::Vec3d(kConeX, 0.0, kConeZ)};
    std::vector<cv::Vec2d> uv(pts.size());
    std::vector<uint8_t> valid(pts.size());

    model->ProjectPoints(pts, uv, valid);
    EXPECT_NEAR(kCx + kTheta30HeightMm / kPixelMm, uv[0][0], 1e-6)
        << "direction pin: forward lookup must be theta -> r_px";
    EXPECT_NEAR(kCy, uv[0][1], 1e-6);
    EXPECT_EQ(1, valid[0]);
}

TEST(LutModel, TableEnd_ClampsLikeRenderer) {
    auto model = MakeCameraModel("table", MakeLutParams());
    const double at_end = kTheta100HeightMm / kPixelMm;
    std::vector<cv::Vec3d> pts{
        cv::Vec3d(std::sin(100.0 * M_PI / 180.0), 0.0,
                  std::cos(100.0 * M_PI / 180.0)),
        cv::Vec3d(std::sin(110.0 * M_PI / 180.0), 0.0,
                  std::cos(110.0 * M_PI / 180.0)),
    };
    std::vector<cv::Vec2d> uv(pts.size());
    std::vector<uint8_t> valid(pts.size());

    model->ProjectPoints(pts, uv, valid);
    EXPECT_NEAR(kCx + at_end, uv[0][0], 1e-6);
    EXPECT_NEAR(kCx + at_end, uv[1][0], 1e-6) << "past-table angle clamps";
    EXPECT_EQ(0, valid[0]);
    EXPECT_EQ(0, valid[1]);
}

TEST(LutModel, InterpolatesBetweenRows) {
    auto model = MakeCameraModel("table", MakeLutParams());
    std::vector<cv::Vec3d> pts{
        cv::Vec3d(std::sin(30.05 * M_PI / 180.0), 0.0,
                  std::cos(30.05 * M_PI / 180.0))};
    std::vector<cv::Vec2d> uv(pts.size());
    std::vector<uint8_t> valid(pts.size());

    model->ProjectPoints(pts, uv, valid);
    const double r_mm = 0.5 * (kTheta30HeightMm + kTheta30Point1HeightMm);
    EXPECT_NEAR(kCx + r_mm / kPixelMm, uv[0][0], 1e-6);
    EXPECT_EQ(1, valid[0]);
}

TEST(LutModel, NonAscendingPoints_Throws) {
    LutParams p = MakeLutParams();
    p.points = {{30.0, 0.5}, {20.0, 0.4}};
    try {
        auto model = MakeCameraModel("table", p);
        (void)model;
        FAIL() << "expected std::invalid_argument for non-ascending table";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find("ascending"), std::string::npos)
            << "what() should mention ascending, got: " << e.what();
    }
}

TEST(LutModel, SinglePoint_Throws) {
    LutParams p = MakeLutParams();
    p.points = {{30.0, 0.5}};
    EXPECT_THROW(MakeCameraModel("table", p), std::invalid_argument);
}

TEST(LutModel, NonPositivePixelSize_Throws) {
    LutParams p = MakeLutParams();
    p.pixel_size_mm = 0.0;
    EXPECT_THROW(MakeCameraModel("table", p), std::invalid_argument);
}

TEST(LutModel, NonPositiveSize_Throws) {
    LutParams p = MakeLutParams();
    p.width = 0;
    try {
        auto model = MakeCameraModel("table", p);
        (void)model;
        FAIL() << "expected std::invalid_argument for width=0";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find("width"), std::string::npos)
            << "what() should mention width, got: " << e.what();
    }
}

TEST(LutModel, BatchMatchesSingle) {
    auto model = MakeCameraModel("table", MakeLutParams());

    std::vector<cv::Vec3d> pts;
    for (int i = 0; i < 50; ++i) {
        const double t = i * 0.63;
        pts.push_back(cv::Vec3d(std::sin(t), std::cos(t * 0.7),
                                std::cos(t) * (i % 2 == 0 ? 1.0 : -1.0)));
    }
    std::vector<cv::Vec2d> uv_batch(pts.size());
    std::vector<uint8_t> valid_batch(pts.size());
    model->ProjectPoints(pts, uv_batch, valid_batch);

    for (size_t i = 0; i < pts.size(); ++i) {
        std::vector<cv::Vec3d> one{pts[i]};
        std::vector<cv::Vec2d> uv_one(one.size());
        std::vector<uint8_t> valid_one(one.size());
        model->ProjectPoints(one, uv_one, valid_one);
        EXPECT_EQ(uv_one[0][0], uv_batch[i][0]) << "point " << i << " u";
        EXPECT_EQ(uv_one[0][1], uv_batch[i][1]) << "point " << i << " v";
        EXPECT_EQ(valid_one[0], valid_batch[i]) << "point " << i << " valid";
    }
}

TEST(LutModel, OutsideRectangle_Invalid) {
    auto model = MakeCameraModel("table", MakeLutParams());
    std::vector<cv::Vec3d> pts{
        cv::Vec3d(std::sin(100.0 * M_PI / 180.0), 0.0,
                  std::cos(100.0 * M_PI / 180.0))};
    std::vector<cv::Vec2d> uv(pts.size());
    std::vector<uint8_t> valid(pts.size());

    model->ProjectPoints(pts, uv, valid);
    EXPECT_EQ(0, valid[0]);
    EXPECT_GT(uv[0][0], static_cast<double>(kWidth));
}
