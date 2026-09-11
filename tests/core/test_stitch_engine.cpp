// Tests for the StitchEngine two-phase API. GL-dependent tests are gated:
// they skip unless AVM_STITCH_TEST_GL=1 or a working OpenGL context is
// detected. The OSGViewRenderer tests live in
// tests/render/test_osg_view_renderer.cpp.

#include <gtest/gtest.h>

#include "geometry/bowl_model.hpp"
#include "core/stitch_engine.hpp"

#include "test_support/gl_fixture.hpp"
#include "test_support/stitch_test_helpers.hpp"

#include <array>
#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

using namespace avm_stitch;
using avm_stitch::test::OSGGLTest;

namespace {

// ---------------------------------------------------------------------------
// Vertical-flip regression helpers. A 2-band red/blue frame is BLIND to the
// flip bug (broken and fixed outputs have identical top/bottom colour
// counts), so use a 6-band direction-sensitive frame instead: band 0 = image
// TOP ... band 5 = image BOTTOM, each a distinct saturated colour.
// ---------------------------------------------------------------------------

// 6 saturated colours, one per horizontal band (BGR order).
const std::vector<cv::Scalar> kBandColours = {
    cv::Scalar(0, 0, 255),     // band 0 (image top):    red
    cv::Scalar(0, 255, 0),     // band 1:                green
    cv::Scalar(255, 0, 0),     // band 2:                blue
    cv::Scalar(0, 255, 255),   // band 3:                yellow
    cv::Scalar(255, 0, 255),   // band 4:                magenta
    cv::Scalar(255, 255, 0),   // band 5 (image bottom): cyan
};

// 640x480 BGR frame split into 6 equal horizontal bands (80 px each).
cv::Mat MakeBandFrame() {
    CameraIntrinsics intr = test::MakeIntrinsics();
    cv::Mat frame(intr.height, intr.width, CV_8UC3);
    const int band_h = intr.height / 6;
    for (int b = 0; b < 6; ++b) {
        frame.rowRange(b * band_h, (b + 1) * band_h).setTo(kBandColours[b]);
    }
    return frame;
}

// Map a BGR pixel to its band (exact match on saturated colours); -1 if the
// pixel is not one of the 6 band colours (background / blend seam).
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

// Dominant band among rows [y0, y1), sampling the centre columns (avoids
// seams at the bowl rim and view borders). Returns -1 if no band-coloured
// pixel is found in the region.
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

// ---------------------------------------------------------------------------
// View-dump helpers (visual inspection): write the 5 rendered views as PNGs
// and report where the sky/ground frame colours land in each view.
// ---------------------------------------------------------------------------

const cv::Scalar kDumpSkyBlue(235, 206, 135);    // BGR light blue (sky)
const cv::Scalar kDumpGroundGreen(0, 220, 0);    // BGR bright green (ground)

// 640x480 BGR frames split vertically: top half = sky blue, bottom half =
// ground green — a simple fisheye-like model (image top looks up/away,
// image bottom looks down at the ground near the car).
std::vector<cv::Mat> MakeSkyGroundFrames(int num_cameras) {
    CameraIntrinsics intr = test::MakeIntrinsics();
    std::vector<cv::Mat> frames;
    for (int i = 0; i < num_cameras; ++i) {
        cv::Mat frame(intr.height, intr.width, CV_8UC3, kDumpGroundGreen);
        frame.rowRange(0, intr.height / 2).setTo(kDumpSkyBlue);
        frames.push_back(frame);
    }
    return frames;
}

// Write the 5 views of one render to dir as {prefix}_{view_name}.png.
// Returns the number of PNG files successfully written (expect 5).
int DumpViews(const std::map<std::string, cv::Mat>& views,
              const std::string& dir, const std::string& prefix) {
    std::filesystem::create_directories(dir);
    int written = 0;
    for (const auto& kv : views) {
        const std::string path = dir + "/" + prefix + "_" + kv.first + ".png";
        if (cv::imwrite(path, kv.second)) ++written;
    }
    return written;
}

// Count pixels within Manhattan distance tol of a colour and report the
// centroid as fractions of the image (y_frac 0=top .. 1=bottom, x_frac
// 0=left .. 1=right). A negative y_frac means the colour was not found.
void PrintColourRegionStats(const cv::Mat& img, const std::string& label,
                             const cv::Scalar& colour, int tol) {
    long count = 0;
    double sum_y = 0, sum_x = 0;
    for (int y = 0; y < img.rows; ++y) {
        const uchar* row = img.ptr<uchar>(y);
        for (int x = 0; x < img.cols; ++x) {
            const uchar* px = row + x * 3;
            const int d = std::abs(px[0] - colour[0]) +
                          std::abs(px[1] - colour[1]) +
                          std::abs(px[2] - colour[2]);
            if (d <= tol) {
                ++count;
                sum_y += y;
                sum_x += x;
            }
        }
    }
    const double y_frac = count ? sum_y / count / img.rows : -1.0;
    const double x_frac = count ? sum_x / count / img.cols : -1.0;
    std::cout << "    " << label << ": count=" << count
              << " y_frac=" << y_frac << " x_frac=" << x_frac << "\n";
}

// Report where the sky (image-top) and ground (image-bottom) colours land.
void PrintSkyGroundStats(const cv::Mat& img, const std::string& view_name) {
    std::cout << "  [" << view_name << " " << img.cols << "x" << img.rows << "]\n";
    PrintColourRegionStats(img, "sky", kDumpSkyBlue, 60);
    PrintColourRegionStats(img, "ground", kDumpGroundGreen, 60);
}

} // namespace

// ---------------------------------------------------------------------------
// StitchEngine two-phase API (validation tests — no GL required).
// Configure is GL-free: the OSG viewer context is created lazily on the first
// RenderFrame.
// ---------------------------------------------------------------------------
TEST(StitchEngineTest, ConfigureN0Throws) {
    StitchEngine engine;
    EXPECT_THROW(engine.Configure({}, {}, BowlParams::Default()),
                 std::invalid_argument);
}

TEST(StitchEngineTest, ConfigureN9Throws) {
    auto intrs = std::vector<CameraIntrinsics>(9, test::MakeIntrinsics());
    auto extrs = test::MakeSideCameras({0, 40, 80, 120, 160, 200, 240, 280, 320});
    StitchEngine engine;
    EXPECT_THROW(engine.Configure(intrs, extrs, BowlParams::Default()),
                 std::invalid_argument);
}

TEST(StitchEngineTest, ConfigureMismatchedSizes) {
    auto intrs = std::vector<CameraIntrinsics>(4, test::MakeIntrinsics());
    auto extrs = test::MakeSideCameras({0, 90, 180, 270});
    extrs.pop_back();   // now 3 extrinsics vs 4 intrinsics
    StitchEngine engine;
    EXPECT_THROW(engine.Configure(intrs, extrs, BowlParams::Default()),
                 std::invalid_argument);
}

TEST(StitchEngineTest, ConfigureMismatchedImageSizesThrows) {
    // The camera texture array is one shared 2D array: all cameras must have
    // the same image size. Camera 1 differs -> Configure must reject it.
    auto intrs = std::vector<CameraIntrinsics>(4, test::MakeIntrinsics());
    intrs[1].width = 320;   // camera 0 is 640x480
    auto extrs = test::MakeSideCameras({0, 90, 180, 270});
    StitchEngine engine;
    EXPECT_THROW(engine.Configure(intrs, extrs, BowlParams::Default()),
                 std::invalid_argument);
}

TEST(StitchEngineTest, ConfigureN1_N8_Success) {
    for (int n = 1; n <= 8; ++n) {
        auto intrs = std::vector<CameraIntrinsics>(n, test::MakeIntrinsics());
        std::vector<double> azimuths;
        for (int i = 0; i < n; ++i) azimuths.push_back(i * 360.0 / n);
        auto extrs = test::MakeSideCameras(azimuths);
        StitchEngine engine;
        EXPECT_NO_THROW(engine.Configure(intrs, extrs, BowlParams::Default()))
            << "Configure failed for N=" << n;
    }
}

TEST(StitchEngineTest, IsConfiguredFalseBefore) {
    StitchEngine engine;
    EXPECT_FALSE(engine.IsConfigured());
}

TEST(StitchEngineTest, IsConfiguredTrueAfter) {
    auto intrs = std::vector<CameraIntrinsics>(4, test::MakeIntrinsics());
    auto extrs = test::MakeSideCameras({0, 90, 180, 270});
    StitchEngine engine;
    engine.Configure(intrs, extrs, BowlParams::Default());
    EXPECT_TRUE(engine.IsConfigured());
}

TEST(StitchEngineTest, GetNumCameras) {
    auto intrs = std::vector<CameraIntrinsics>(4, test::MakeIntrinsics());
    auto extrs = test::MakeSideCameras({0, 90, 180, 270});
    StitchEngine engine;
    engine.Configure(intrs, extrs, BowlParams::Default());
    EXPECT_EQ(4, engine.GetNumCameras());
}

TEST(StitchEngineTest, RenderFrameBeforeConfigureThrows) {
    StitchEngine engine;
    EXPECT_THROW(engine.RenderFrame({}), std::runtime_error);
}

TEST(StitchEngineTest, RenderFrameWrongImageCount) {
    auto intrs = std::vector<CameraIntrinsics>(4, test::MakeIntrinsics());
    auto extrs = test::MakeSideCameras({0, 90, 180, 270});
    StitchEngine engine;
    engine.Configure(intrs, extrs, BowlParams::Default());

    auto frames = test::MakeColourFrames(3);   // 3 images for a 4-camera engine
    EXPECT_THROW(engine.RenderFrame(frames), std::invalid_argument);
}

TEST(StitchEngineTest, Configure_BowlParamsCarLengthWidth) {
    // Truck-sized bowl params must configure without throwing.
    BowlParams bowl = BowlParams::Default();
    bowl.car_length = 18.0;
    bowl.car_width = 2.5;
    bowl.r_outter = 25.0;
    bowl.step = 1.0;   // coarse grid keeps the test fast

    auto intrs = std::vector<CameraIntrinsics>(4, test::MakeIntrinsics());
    auto extrs = test::MakeSideCameras({0, 90, 180, 270});
    StitchEngine engine;
    EXPECT_NO_THROW(engine.Configure(intrs, extrs, bowl));
    EXPECT_TRUE(engine.IsConfigured());
}

TEST(StitchEngineTest, ConfigureStepZeroThrows) {
    // step <= 0 makes the grid degenerate (rows = r_outter/step blows up or
    // divides by zero) — reject it with a clear error instead of UB.
    auto intrs = std::vector<CameraIntrinsics>(4, test::MakeIntrinsics());
    auto extrs = test::MakeSideCameras({0, 90, 180, 270});
    BowlParams bowl = BowlParams::Default();
    bowl.step = 0.0;
    StitchEngine engine;
    EXPECT_THROW(engine.Configure(intrs, extrs, bowl), std::invalid_argument);
}

TEST(StitchEngineTest, ConfigureNonPositiveRoutterThrows) {
    // r_outter <= 0 would produce an empty/negative-extent bowl.
    auto intrs = std::vector<CameraIntrinsics>(4, test::MakeIntrinsics());
    auto extrs = test::MakeSideCameras({0, 90, 180, 270});
    BowlParams bowl = BowlParams::Default();
    bowl.r_outter = 0.0;
    StitchEngine engine;
    EXPECT_THROW(engine.Configure(intrs, extrs, bowl), std::invalid_argument);
}

TEST(StitchEngineTest, ConfigureNegativeKSlopThrows) {
    auto intrs = std::vector<CameraIntrinsics>(4, test::MakeIntrinsics());
    auto extrs = test::MakeSideCameras({0, 90, 180, 270});
    BowlParams bowl = BowlParams::Default();
    bowl.k_slop = -0.1;
    StitchEngine engine;
    EXPECT_THROW(engine.Configure(intrs, extrs, bowl), std::invalid_argument);
}

TEST(StitchEngineTest, ConfigureNegativeZExpThrows) {
    // z_exp < 0 makes the bowl rise formula (z = k_slop * rd^z_exp) blow up
    // for rd < 1 — reject it with a clear error instead of emitting NaN/inf
    // vertex positions into the model.
    auto intrs = std::vector<CameraIntrinsics>(4, test::MakeIntrinsics());
    auto extrs = test::MakeSideCameras({0, 90, 180, 270});
    BowlParams bowl = BowlParams::Default();
    bowl.z_exp = -1.0;
    StitchEngine engine;
    EXPECT_THROW(engine.Configure(intrs, extrs, bowl), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// StitchEngine GL end-to-end tests (gated).
// ---------------------------------------------------------------------------
TEST_F(OSGGLTest, StitchEngineGL_RenderFrame_Returns5Views) {
    auto intrs = std::vector<CameraIntrinsics>(4, test::MakeIntrinsics());
    auto extrs = test::MakeLookAtCameras(4);
    StitchEngine engine;
    engine.Configure(intrs, extrs, BowlParams::Default());

    auto views = engine.RenderFrame(test::MakeColourFrames(4));
    ASSERT_EQ(5u, views.size());
    EXPECT_NE(views.end(), views.find("topdown"));
    EXPECT_NE(views.end(), views.find("3d_front"));
    EXPECT_NE(views.end(), views.find("3d_rear"));
    EXPECT_NE(views.end(), views.find("3d_left"));
    EXPECT_NE(views.end(), views.find("3d_right"));
}

TEST_F(OSGGLTest, StitchEngineGL_RenderFrame_OutputSize) {
    auto intrs = std::vector<CameraIntrinsics>(4, test::MakeIntrinsics());
    auto extrs = test::MakeLookAtCameras(4);
    StitchEngine engine;
    engine.Configure(intrs, extrs, BowlParams::Default());

    auto views = engine.RenderFrame(test::MakeColourFrames(4));
    ASSERT_EQ(5u, views.size());
    for (const auto& kv : views) {
        EXPECT_EQ(960, kv.second.cols) << kv.first;
        EXPECT_EQ(640, kv.second.rows) << kv.first;
        EXPECT_EQ(CV_8UC3, kv.second.type()) << kv.first;
    }
}

TEST_F(OSGGLTest, StitchEngineGL_RenderFrame_NonBlackOutput) {
    auto intrs = std::vector<CameraIntrinsics>(4, test::MakeIntrinsics());
    auto extrs = test::MakeLookAtCameras(4);
    StitchEngine engine;
    engine.Configure(intrs, extrs, BowlParams::Default());

    auto views = engine.RenderFrame(test::MakeColourFrames(4));
    ASSERT_EQ(5u, views.size());
    // The bowl ground visible to the look-at cameras must render camera
    // colours in EVERY view — not just the clear colour / black. Background
    // clear colour is RGB(30,36,50), a low-saturation colour, so vivid
    // (mx-mn>100) pixels prove the texture upload + mapping shader ran.
    for (const auto& kv : views) {
        EXPECT_GT(test::CountVividPixels(kv.second), 0) << kv.first;
    }
}

TEST_F(OSGGLTest, StitchEngineGL_RenderFrame_N8Cameras) {
    auto intrs = std::vector<CameraIntrinsics>(8, test::MakeIntrinsics());
    auto extrs = test::MakeLookAtCameras(8);
    StitchEngine engine;
    engine.Configure(intrs, extrs, BowlParams::Default());

    auto views = engine.RenderFrame(test::MakeColourFrames(8));
    ASSERT_EQ(5u, views.size());
    int total_vivid = 0;
    for (const auto& kv : views) total_vivid += test::CountVividPixels(kv.second);
    EXPECT_GT(total_vivid, 0);
}

// ---------------------------------------------------------------------------
// Vertical-flip regression: the OSG renderer must NOT mirror camera images.
// The reference subprocess stitcher flips the FBO readback AND the UV content;
// both flips are missing in the OSG path, so the output is vertically
// mirrored. A single camera at +X looking at the bowl centre (N=1) with a
// 6-band direction-sensitive frame exposes the bug: the +X near-camera ground
// (camera image band 5) must render at the TOP of the topdown view.
// ---------------------------------------------------------------------------
TEST_F(OSGGLTest, StitchEngineGL_TopdownOutputNotVerticallyFlipped) {
    auto intrs = std::vector<CameraIntrinsics>{test::MakeIntrinsics()};
    auto extrs =
        std::vector<CameraExtrinsics>{test::MakeLookAtCamera(0.0, 4.0, 2.0, 0.0)};
    StitchEngine engine;
    engine.Configure(intrs, extrs, BowlParams::Default());

    auto views = engine.RenderFrame({MakeBandFrame()});
    auto it = views.find("topdown");
    ASSERT_NE(views.end(), it);
    const cv::Mat& topdown = it->second;

    // The +X side of the bowl (nearest the single camera) renders at the TOP
    // of the topdown output. That ground content is imaged by the camera's
    // BOTTOM band (band 5, cyan). A vertically mirrored renderer samples the
    // wrong band there (band 3, yellow) — this test fails before the fix.
    // Rows 0..80 are background (the topdown frustum's outer rim is above the
    // bowl's 12.6 m edge); the +X near-camera band-5 ground sits at rows
    // 80..140 of the fixed output.
    EXPECT_EQ(5, DominantBand(topdown, 80, 140))
        << "topdown output top bowl rows must sample camera image band 5";
}

TEST_F(OSGGLTest, StitchEngineGL_FrontViewNotVerticallyFlipped) {
    auto intrs = std::vector<CameraIntrinsics>{test::MakeIntrinsics()};
    auto extrs =
        std::vector<CameraExtrinsics>{test::MakeLookAtCamera(0.0, 4.0, 2.0, 0.0)};
    StitchEngine engine;
    engine.Configure(intrs, extrs, BowlParams::Default());

    auto views = engine.RenderFrame({MakeBandFrame()});
    auto it = views.find("3d_front");
    ASSERT_NE(views.end(), it);
    const cv::Mat& front = it->second;

    // The far rim (imaged near the camera's image TOP) renders in the upper
    // portion of the 3d_front output; the near flat ground (camera image
    // BOTTOM) renders in the lower portion. A mirrored renderer swaps these.
    // The 40-px hard gate (feather_px=40) excludes the outermost fisheye rim,
    // shifting the visible portion one band inward (band 1 instead of 0 at
    // top, band 4 instead of 5 at bottom). Scan wider ranges for robustness.
    EXPECT_EQ(1, DominantBand(front, 60, 240))
        << "3d_front output upper rows must sample camera image band 1 (was band 0 before 40-px hard gate)";
    EXPECT_EQ(4, DominantBand(front, 400, 600))
        << "3d_front output bottom rows must sample camera image band 4 (was band 5 before 40-px hard gate)";
}

// ---------------------------------------------------------------------------
// Visual-inspection dump: render the 5 views with realistic-ish camera
// layouts + frames and write PNGs to /tmp/opencode/stitch-views so we can
// eyeball the bowl (ground near the car, sky at the rim, black seams between
// camera blocks). Writes to /tmp only — harmless in CI.
// ---------------------------------------------------------------------------
TEST_F(OSGGLTest, StitchEngineGL_DumpViewsForInspection) {
    const std::string dir = "/tmp/opencode/stitch-views";
    std::filesystem::create_directories(dir);
    const auto intrs = std::vector<CameraIntrinsics>(4, test::MakeIntrinsics());

    // 1) Realistic AVM ring: 4 look-at cameras at radius 4m, height 2m, each
    //    aimed at the bowl centre. Sky/ground split frames.
    {
        auto extrs = test::MakeLookAtCameras(4);
        StitchEngine engine;
        engine.Configure(intrs, extrs, BowlParams::Default());
        auto views = engine.RenderFrame(MakeSkyGroundFrames(4));
        ASSERT_EQ(5u, views.size());
        EXPECT_EQ(5, DumpViews(views, dir, "lookat_skyground"));
        std::cout << "lookat_skyground views:\n";
        for (const auto& kv : views) PrintSkyGroundStats(kv.second, kv.first);
    }

    // 2) Same look-at cameras, one distinct colour per camera — shows the
    //    camera block layout and any black seams between blocks.
    {
        auto extrs = test::MakeLookAtCameras(4);
        StitchEngine engine;
        engine.Configure(intrs, extrs, BowlParams::Default());
        auto views = engine.RenderFrame(test::MakeColourFrames(4));
        ASSERT_EQ(5u, views.size());
        EXPECT_EQ(5, DumpViews(views, dir, "lookat_colour"));
    }

    // 3) Original side cameras (identity rotation, radius 3m, height 1m) with
    //    the same sky/ground frames, for comparison.
    {
        auto extrs = test::MakeSideCameras({0, 90, 180, 270});
        StitchEngine engine;
        engine.Configure(intrs, extrs, BowlParams::Default());
        auto views = engine.RenderFrame(MakeSkyGroundFrames(4));
        ASSERT_EQ(5u, views.size());
        EXPECT_EQ(5, DumpViews(views, dir, "side_skyground"));
        std::cout << "side_skyground views:\n";
        for (const auto& kv : views) PrintSkyGroundStats(kv.second, kv.first);
    }
}
