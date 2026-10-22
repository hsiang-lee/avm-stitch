
#include <gtest/gtest.h>

#include "bowl_model/bowl_model.hpp"
#include "core/stitch_engine.hpp"
#include "render/osg_view_renderer.hpp"

#include "test_support/gl_fixture.hpp"
#include "test_support/stitch_test_helpers.hpp"

#include <array>
#include <string>
#include <vector>

using namespace avm_stitch;
using avm_stitch::test::OSGGLTest;

namespace {



const std::vector<cv::Scalar> kBandColours = {
    cv::Scalar(0, 0, 255),         cv::Scalar(0, 255, 0),         cv::Scalar(255, 0, 0),         cv::Scalar(0, 255, 255),       cv::Scalar(255, 0, 255),       cv::Scalar(255, 255, 0),   };

cv::Mat MakeBandFrame() {
    KbParams intr = test::MakeIntrinsics();
    cv::Mat frame(intr.height, intr.width, CV_8UC3);
    const int band_h = intr.height / 6;
    for (int b = 0; b < 6; ++b) {
        frame.rowRange(b * band_h, (b + 1) * band_h).setTo(kBandColours[b]);
    }
    return frame;
}

int MatchBand(const cv::Vec3b& px) {
    for (int b = 0; b < 6; ++b) {
        if (px[0] == static_cast<uchar>(kBandColours[b][0]) &&
            px[1] == static_cast<uchar>(kBandColours[b][1]) &&
            px[2] == static_cast<uchar>(kBandColours[b][2])) {
            return b;
        }
    }
    return -1;
}

int DominantBand(const cv::Mat& img, int y0, int y1) {
    std::array<int, 6> counts{};
    int matched = 0;
    const int x0 = std::max(0, img.cols / 2 - 30);
    const int x1 = std::min(img.cols, img.cols / 2 + 30);
    for (int y = y0; y < std::min(y1, img.rows); ++y) {
        const uchar* row = img.ptr<uchar>(y);
        for (int x = x0; x < x1; ++x) {
            const uchar* px = row + x * 3;
            int b = MatchBand(cv::Vec3b(px[0], px[1], px[2]));
            if (b >= 0) {
                ++counts[b];
                ++matched;
            }
        }
    }
    if (matched == 0) return -1;
    return static_cast<int>(std::max_element(counts.begin(), counts.end()) -
                            counts.begin());
}

}
TEST(StitchEngineTest, ConfigureN0Throws) {
    StitchEngine engine;
    EXPECT_THROW(engine.Configure({}, test::MakeCarBowl()),
                 std::invalid_argument);
}

TEST(StitchEngineTest, ConfigureN9Throws) {
    StitchEngine engine;
    EXPECT_THROW(engine.Configure(test::WrapKbSideCameras({0, 40, 80, 120, 160,
                                                           200, 240, 280, 320}),
                                  test::MakeCarBowl()),
                 std::invalid_argument);
}

TEST(StitchEngineTest, ConfigureMismatchedImageSizesThrows) {
    auto cams = test::WrapKbSideCameras({0, 90, 180, 270});
    cams[1] = Camera(cams[1].extrinsics,
                     MakeCameraModel("kb",
                                     KbParams{320, 480, 300.0, 300.0, 320.0, 240.0,
                                              0, 0, 0, 0}));
    StitchEngine engine;
    EXPECT_THROW(engine.Configure(std::move(cams), test::MakeCarBowl()),
                 std::invalid_argument);
}

TEST(StitchEngineTest, ConfigureNullCameraModelThrows) {
    // A null model may hide inside the caller's vector<Camera>; Configure owns
    // the models, so it must reject a null entry with a clear null-model
    // message instead of dereferencing it during the bake. All cameras are
    // otherwise valid (N and image-size checks pass), so execution must reach
    // the per-camera Valid() gate.
    auto cams = test::WrapKbSideCameras({0, 90, 180, 270});
    cams[2] = Camera(cams[2].extrinsics, nullptr);   // null model (Valid() false)
    StitchEngine engine;
    EXPECT_THROW(
        {
            try {
                engine.Configure(std::move(cams), test::MakeCarBowl());
            } catch (const std::invalid_argument& e) {
                EXPECT_NE(std::string(e.what())
                              .find(" has no model (Camera.model is null)"),
                          std::string::npos)
                    << "what() should mention the null model, got: " << e.what();
                throw;   // rethrow so EXPECT_THROW sees the exception
            }
        },
        std::invalid_argument);
}

TEST(StitchEngineTest, ConfigureN1_N8_Success) {
    for (int n = 1; n <= 8; ++n) {
        std::vector<double> azimuths;
        for (int i = 0; i < n; ++i) azimuths.push_back(i * 360.0 / n);
        StitchEngine engine;
        EXPECT_NO_THROW(engine.Configure(test::WrapKbSideCameras(azimuths),
                                         test::MakeCarBowl()))
            << "Configure failed for N=" << n;
    }
}

TEST(StitchEngineTest, IsConfiguredFalseBefore) {
    StitchEngine engine;
    EXPECT_FALSE(engine.IsConfigured());
}

TEST(StitchEngineTest, IsConfiguredTrueAfter) {
    StitchEngine engine;
    engine.Configure(test::WrapKbSideCameras({0, 90, 180, 270}),
                     test::MakeCarBowl());
    EXPECT_TRUE(engine.IsConfigured());
}

TEST(StitchEngineTest, GetNumCameras) {
    StitchEngine engine;
    engine.Configure(test::WrapKbSideCameras({0, 90, 180, 270}),
                     test::MakeCarBowl());
    EXPECT_EQ(4, engine.GetNumCameras());
}

TEST(StitchEngineTest, RenderFrameBeforeConfigureThrows) {
    StitchEngine engine;
    EXPECT_THROW(engine.RenderFrame({}), std::runtime_error);
}

TEST(StitchEngineTest, RenderFrameWrongImageCount) {
    StitchEngine engine;
    engine.Configure(test::WrapKbSideCameras({0, 90, 180, 270}),
                     test::MakeCarBowl());

    auto frames = test::MakeColourFrames(3);       EXPECT_THROW(engine.RenderFrame(frames), std::invalid_argument);
}

TEST(StitchEngineTest, Configure_BowlParamsCarLengthWidth) {
        BowlParams bowl = test::MakeCarBowl()
        .with_car_length(18.0)
        .with_car_width(2.5)
        .with_r_outter(25.0)
        .with_step(1.0);
    StitchEngine engine;
    EXPECT_NO_THROW(engine.Configure(test::WrapKbSideCameras({0, 90, 180, 270}),
                                     bowl));
    EXPECT_TRUE(engine.IsConfigured());
}

TEST(StitchEngineTest, ConfigureStepZeroThrows) {
    BowlParams bowl = test::MakeCarBowl().with_step(0.0);
    StitchEngine engine;
    EXPECT_THROW(engine.Configure(test::WrapKbSideCameras({0, 90, 180, 270}),
                                  bowl),
                 std::invalid_argument);
}

TEST(StitchEngineTest, ConfigureNonPositiveRoutterThrows) {
    BowlParams bowl = test::MakeCarBowl().with_r_outter(0.0);
    StitchEngine engine;
    EXPECT_THROW(engine.Configure(test::WrapKbSideCameras({0, 90, 180, 270}),
                                  bowl),
                 std::invalid_argument);
}

TEST(StitchEngineTest, ConfigureNegativeKSlopThrows) {
    BowlParams bowl = test::MakeCarBowl().with_k_slop(-0.1);
    StitchEngine engine;
    EXPECT_THROW(engine.Configure(test::WrapKbSideCameras({0, 90, 180, 270}),
                                  bowl),
                 std::invalid_argument);
}

TEST(StitchEngineTest, ConfigureNegativeZExpThrows) {
    BowlParams bowl = test::MakeCarBowl().with_z_exp(-1.0);
    StitchEngine engine;
    EXPECT_THROW(engine.Configure(test::WrapKbSideCameras({0, 90, 180, 270}),
                                  bowl),
                 std::invalid_argument);
}

TEST_F(OSGGLTest, StitchEngineGL_RenderFrame_Returns4Views) {
    StitchEngine engine;
    engine.Configure(
        test::WrapKbCameras(std::vector<KbParams>(4, test::MakeIntrinsics()),
                            test::MakeLookAtCameras(4)),
        test::MakeCarBowl());

    auto views = engine.RenderFrame(test::MakeColourFrames(4));
    ASSERT_EQ(4u, views.size());
    EXPECT_NE(views.end(), views.find("3d_front"));
    EXPECT_NE(views.end(), views.find("3d_rear"));
    EXPECT_NE(views.end(), views.find("3d_left"));
    EXPECT_NE(views.end(), views.find("3d_right"));
    EXPECT_EQ(views.end(), views.find("topdown"));
}

TEST_F(OSGGLTest, ConfigureKbPath_Configures_AndRenders4Views) {
                        StitchEngine engine;
    engine.Configure(
        test::WrapKbCameras(std::vector<KbParams>(4, test::MakeIntrinsics()),
                            test::MakeLookAtCameras(4)),
        test::MakeCarBowl());
    ASSERT_TRUE(engine.IsConfigured());

    auto frames = test::MakeColourFrames(4);
    auto views = engine.RenderFrame(frames);
        EXPECT_EQ(4, views.size());
}

TEST_F(OSGGLTest, StitchEngineGL_RenderFrame_OutputSize) {
    StitchEngine engine;
    engine.Configure(
        test::WrapKbCameras(std::vector<KbParams>(4, test::MakeIntrinsics()),
                            test::MakeLookAtCameras(4)),
        test::MakeCarBowl());

    auto views = engine.RenderFrame(test::MakeColourFrames(4));
    ASSERT_EQ(4u, views.size());
    for (const auto& kv : views) {
        EXPECT_EQ(960, kv.second.cols) << kv.first;
        EXPECT_EQ(640, kv.second.rows) << kv.first;
        EXPECT_EQ(CV_8UC3, kv.second.type()) << kv.first;
    }
}

TEST_F(OSGGLTest, StitchEngineGL_RenderFrame_NonBlackOutput) {
    StitchEngine engine;
    engine.Configure(
        test::WrapKbCameras(std::vector<KbParams>(4, test::MakeIntrinsics()),
                            test::MakeLookAtCameras(4)),
        test::MakeCarBowl());

    auto views = engine.RenderFrame(test::MakeColourFrames(4));
    ASSERT_EQ(4u, views.size());
                    for (const auto& kv : views) {
        EXPECT_GT(test::CountVividPixels(kv.second), 0) << kv.first;
    }
}

TEST_F(OSGGLTest, StitchEngineGL_RenderFrame_N8Cameras) {
    StitchEngine engine;
    engine.Configure(
        test::WrapKbCameras(std::vector<KbParams>(8, test::MakeIntrinsics()),
                            test::MakeLookAtCameras(8)),
        test::MakeCarBowl());

    auto views = engine.RenderFrame(test::MakeColourFrames(8));
    ASSERT_EQ(4u, views.size());
    int total_vivid = 0;
    for (const auto& kv : views) total_vivid += test::CountVividPixels(kv.second);
    EXPECT_GT(total_vivid, 0);
}

TEST_F(OSGGLTest, StitchEngineGL_FrontViewNotVerticallyFlipped) {
    BowlParams bowl = test::MakeCarBowl().with_car_center(cv::Vec2d(0.0, 0.0));
    StitchEngine engine;
    engine.Configure(
        test::WrapKbCameras(std::vector<KbParams>(1, test::MakeIntrinsics()),
                            std::vector<CameraExtrinsics>{
                                test::MakeLookAtCamera(0.0, 4.0, 2.0, 0.0)}),
        bowl);

    auto views = engine.RenderFrame({MakeBandFrame()});
    auto it = views.find("3d_front");
    ASSERT_NE(views.end(), it);
    const cv::Mat& front = it->second;

                            EXPECT_EQ(1, DominantBand(front, 60, 240))
        << "3d_front output upper rows must sample camera image band 1 (was band 0 before 40-px hard gate)";
    EXPECT_EQ(4, DominantBand(front, 400, 600))
        << "3d_front output bottom rows must sample camera image band 4 (was band 5 before 40-px hard gate)";
}

TEST(RegressionTest, PublicApiHeadersAreSelfContained) {
            const BowlParams bowl = DeriveBowlParams(5.6, 2.2, 2.9);
    EXPECT_DOUBLE_EQ(12.6, bowl.r_outter());

    auto poses = bowl.DefaultViewPoses();
    EXPECT_EQ(4u, poses.size());
}
