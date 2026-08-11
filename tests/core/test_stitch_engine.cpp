// Tests for the OSG view renderer (offscreen 5-view rendering) and the
// StitchEngine two-phase API. GL-dependent tests are gated: they skip unless
// AVM_STITCH_TEST_GL=1 or a working OpenGL context is detected.

#include <gtest/gtest.h>

#include "geometry/bowl_model.hpp"
#include "render/egl_headless_context.hpp"
#include "geometry/mesh_bake.hpp"
#include "render/osg_scene.hpp"
#include "render/osg_view.hpp"
#include "core/stitch_engine.hpp"
#include "util/view_config.hpp"

#include <osg/Camera>
#include <osg/Geode>
#include <osg/Image>
#include <osg/Texture2DArray>
#include <osgViewer/Viewer>

#include <GL/gl.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using namespace avm_worker::avm_stitch;

namespace {

CameraIntrinsics MakeIntrinsics() {
    // 640x480 with a moderate focal length: a ring of bowl-rim vertices near
    // each side camera projects inside the image, so the bake has visible
    // cameras and the mapping shader has real texture work to do.
    return {640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
}

// N identity-rotation cameras at azimuths (deg), radius 3m, height 1m.
// Only bowl vertices with z > 1 (the rising rim) are forward-facing.
std::vector<CameraExtrinsics> MakeSideCameras(
    const std::vector<double>& azimuths_deg) {
    std::vector<CameraExtrinsics> cams;
    for (double az : azimuths_deg) {
        double rad = az * M_PI / 180.0;
        cv::Vec3d pos(std::cos(rad) * 3.0, std::sin(rad) * 3.0, 1.0);
        cams.push_back({cv::Matx33d::eye(), pos});
    }
    return cams;
}

BowlMesh MakeCoarseBowl(double r_outter) {
    BowlParams bowl = BowlParams::Default();
    bowl.step = 1.0;        // coarse grid for test speed
    bowl.r_outter = r_outter;
    return CreateBowlMesh(bowl);
}

// Bake a coarse bowl for N side cameras and build the OSG scene for it.
// Returns the scene root plus the texture array (to upload synthetic frames).
struct TestScene {
    osg::ref_ptr<osg::Group> root;
    osg::ref_ptr<osg::Texture2DArray> texture;
    bool has_visible_vertices = false;
};

TestScene BuildTestSceneWithExtrinsics(int num_cameras,
                                       const std::vector<CameraExtrinsics>& extrs) {
    TestScene ts;
    auto mesh = MakeCoarseBowl(12.6);
    std::vector<CameraIntrinsics> intrs(num_cameras, MakeIntrinsics());
    auto baked = BakeMesh(mesh, intrs, extrs);

    for (const auto& bv : baked.vertices) {
        double sum = 0.0;
        for (int j = 0; j < num_cameras; ++j) sum += bv.weight[j];
        if (sum > 1e-9) { ts.has_visible_vertices = true; break; }
    }

    ts.root = BuildOSGScene(mesh, baked, num_cameras, intrs[0].width, intrs[0].height);

    // Pull the camera texture array out of the bowl geode's stateset so the
    // test can upload synthetic frames (RenderFrame does this internally).
    if (ts.root.valid() && ts.root->getNumChildren() > 0) {
        if (auto* geode = dynamic_cast<osg::Geode*>(ts.root->getChild(0))) {
            if (auto* ss = geode->getStateSet()) {
                ts.texture = dynamic_cast<osg::Texture2DArray*>(
                    ss->getTextureAttribute(0, osg::StateAttribute::TEXTURE));
            }
        }
    }
    return ts;
}

TestScene BuildTestScene(int num_cameras) {
    std::vector<double> azimuths;
    for (int i = 0; i < num_cameras; ++i) azimuths.push_back(i * 360.0 / num_cameras);
    return BuildTestSceneWithExtrinsics(num_cameras, MakeSideCameras(azimuths));
}

// Synthetic BGR frames, one distinct colour per camera (BGR order).
// Size matches MakeIntrinsics() so texture upload has no dimension mismatch.
std::vector<cv::Mat> MakeColourFrames(int num_cameras) {
    const cv::Scalar colours[4] = {
        cv::Scalar(0, 0, 255),     // camera 0: red
        cv::Scalar(0, 255, 0),     // camera 1: green
        cv::Scalar(255, 0, 0),     // camera 2: blue
        cv::Scalar(0, 255, 255),   // camera 3: yellow
    };
    CameraIntrinsics intr = MakeIntrinsics();
    std::vector<cv::Mat> frames;
    for (int i = 0; i < num_cameras; ++i) {
        frames.push_back(cv::Mat(intr.height, intr.width, CV_8UC3, colours[i % 4]));
    }
    return frames;
}

// Cameras mounted on a ring that LOOK AT the ground around the car (realistic
// AVM setup): the flat bowl centre is visible, so RenderFrame output is not
// black. Camera at (r*cos az, r*sin az, h) looking toward (0, 0, target_z).
CameraExtrinsics MakeLookAtCamera(double az_deg, double r, double h,
                                  double target_z) {
    double rad = az_deg * M_PI / 180.0;
    cv::Vec3d pos(r * std::cos(rad), r * std::sin(rad), h);
    cv::Vec3d target(0.0, 0.0, target_z);
    cv::Vec3d f = target - pos;
    f = f / cv::norm(f);
    cv::Vec3d up(0.0, 0.0, 1.0);
    cv::Vec3d right = f.cross(up);
    right = right / cv::norm(right);
    cv::Vec3d u = right.cross(f);
    cv::Vec3d down = -u;   // camera y axis points DOWN (OpenCV convention)
    // T_wc convention: columns = [right | down | forward], y axis down per OpenCV.
    cv::Matx33d R(right[0], down[0], f[0],
                  right[1], down[1], f[1],
                  right[2], down[2], f[2]);
    return {R, pos};
}

// N look-at cameras evenly spaced around the car.
std::vector<CameraExtrinsics> MakeLookAtCameras(int num_cameras) {
    std::vector<CameraExtrinsics> cams;
    for (int i = 0; i < num_cameras; ++i) {
        cams.push_back(MakeLookAtCamera(i * 360.0 / num_cameras, 4.0, 2.0, 0.0));
    }
    return cams;
}

// Count vivid (high-saturation) pixels — the camera-colour regions of a view.
int CountVividPixels(const cv::Mat& img) {
    int vivid = 0;
    for (int y = 0; y < img.rows; ++y) {
        const uchar* row = img.ptr<uchar>(y);
        for (int x = 0; x < img.cols; ++x) {
            const uchar* px = row + x * 3;
            int mn = std::min({px[0], px[1], px[2]});
            int mx = std::max({px[0], px[1], px[2]});
            if (mx - mn > 100) ++vivid;
        }
    }
    return vivid;
}

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
    CameraIntrinsics intr = MakeIntrinsics();
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
    CameraIntrinsics intr = MakeIntrinsics();
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
// GL fixture: skip the whole suite when no OpenGL context is available.
// ---------------------------------------------------------------------------
class OSGGLTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!IsGLContextAvailable()) {
            GTEST_SKIP() << "OpenGL context not available (set AVM_STITCH_TEST_GL=1 to force)";
        }
    }

    static bool IsGLContextAvailable() {
        static const bool available = DetectGLContext();
        return available;
    }

    static bool DetectGLContext() {
        const char* env = std::getenv("AVM_STITCH_TEST_GL");
        if (env) return std::atoi(env) == 1;
        // Probe: a headless EGL surfaceless context (no X server, no DISPLAY)
        // whose GL_VERSION must be queryable once current.
        try {
            auto gc = CreateEGLHeadlessContext(1, 1);
            if (!gc->realizeImplementation()) return false;
            bool ok = gc->valid() && gc->makeCurrentImplementation();
            if (ok) {
                const char* ver =
                    reinterpret_cast<const char*>(glGetString(GL_VERSION));
                ok = ver != nullptr;
            }
            gc->closeImplementation();
            return ok;
        } catch (...) {
            return false;
        }
    }
};

// ---------------------------------------------------------------------------
// Task 7: OSGViewRenderer (offscreen 5-view rendering).
// ---------------------------------------------------------------------------
TEST_F(OSGGLTest, OSGViewRenderer_InitializeDoesNotCrash) {
    auto ts = BuildTestScene(4);
    ASSERT_TRUE(ts.root.valid());
    ASSERT_TRUE(ts.has_visible_vertices);   // otherwise the test proves nothing

    OSGViewRenderer renderer;
    EXPECT_NO_THROW(renderer.Initialize(ts.root, DefaultViewPoses(), 960, 640));
}

TEST_F(OSGGLTest, OSGViewRenderer_RenderAllViews_Returns5Mats) {
    auto ts = BuildTestScene(4);
    ASSERT_TRUE(ts.root.valid());
    ASSERT_TRUE(ts.texture.valid());
    ASSERT_TRUE(ts.has_visible_vertices);

    OSGViewRenderer renderer;
    renderer.Initialize(ts.root, DefaultViewPoses(), 960, 640);
    auto views = renderer.RenderAllViews();

    ASSERT_EQ(5u, views.size());
    EXPECT_NE(views.end(), views.find("topdown"));
    EXPECT_NE(views.end(), views.find("3d_front"));
    EXPECT_NE(views.end(), views.find("3d_rear"));
    EXPECT_NE(views.end(), views.find("3d_left"));
    EXPECT_NE(views.end(), views.find("3d_right"));
}

TEST_F(OSGGLTest, OSGViewRenderer_RenderAllViews_CorrectSize) {
    auto ts = BuildTestScene(4);
    ASSERT_TRUE(ts.root.valid());
    ASSERT_TRUE(ts.has_visible_vertices);

    OSGViewRenderer renderer;
    renderer.Initialize(ts.root, DefaultViewPoses(), 960, 640);
    auto views = renderer.RenderAllViews();
    ASSERT_EQ(5u, views.size());

    for (const auto& kv : views) {
        EXPECT_EQ(960, kv.second.cols) << kv.first;
        EXPECT_EQ(640, kv.second.rows) << kv.first;
        EXPECT_EQ(CV_8UC3, kv.second.type()) << kv.first;
    }
}

TEST_F(OSGGLTest, OSGViewRenderer_RenderAllViews_DrawsSceneNotJustBackground) {
    // Look-at cameras cover the flat bowl centre too, so EVERY view (including
    // topdown, which looks straight down at the centre) samples textured
    // pixels. The old side-camera setup only textures the z>1 rim, which the
    // topdown frustum never sees.
    auto ts = BuildTestSceneWithExtrinsics(4, MakeLookAtCameras(4));
    ASSERT_TRUE(ts.root.valid());
    ASSERT_TRUE(ts.texture.valid());
    ASSERT_TRUE(ts.has_visible_vertices);

    // Upload distinct colours so the bowl rim renders vivid pixels.
    UploadCameraImages(ts.texture.get(), MakeColourFrames(4));

    OSGViewRenderer renderer;
    renderer.Initialize(ts.root, DefaultViewPoses(), 960, 640);
    auto views = renderer.RenderAllViews();
    ASSERT_EQ(5u, views.size());

    // Background clear colour is RGB(30,36,50) — a low-saturation colour. The
    // bowl must render vivid camera-colour pixels in EVERY view, proving the
    // texture array upload + mapping shader actually ran (not just the FBO
    // clear). Each view needs its own Geode referencing the shared Geometry —
    // the single-geode approach reparents it to the last camera, leaving the
    // other four views blank.
    for (const auto& kv : views) {
        EXPECT_GT(CountVividPixels(kv.second), 0) << kv.first;
    }
}

// ---------------------------------------------------------------------------
// Task 8: StitchEngine two-phase API (validation tests — no GL required).
// Configure is GL-free: the OSG viewer context is created lazily on the first
// RenderFrame.
// ---------------------------------------------------------------------------
TEST(StitchEngineTest, ConfigureN0Throws) {
    StitchEngine engine;
    EXPECT_THROW(engine.Configure({}, {}, BowlParams::Default()),
                 std::invalid_argument);
}

TEST(StitchEngineTest, ConfigureN9Throws) {
    auto intrs = std::vector<CameraIntrinsics>(9, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 40, 80, 120, 160, 200, 240, 280, 320});
    StitchEngine engine;
    EXPECT_THROW(engine.Configure(intrs, extrs, BowlParams::Default()),
                 std::invalid_argument);
}

TEST(StitchEngineTest, ConfigureMismatchedSizes) {
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 90, 180, 270});
    extrs.pop_back();   // now 3 extrinsics vs 4 intrinsics
    StitchEngine engine;
    EXPECT_THROW(engine.Configure(intrs, extrs, BowlParams::Default()),
                 std::invalid_argument);
}

TEST(StitchEngineTest, ConfigureMismatchedImageSizesThrows) {
    // The camera texture array is one shared 2D array: all cameras must have
    // the same image size. Camera 1 differs -> Configure must reject it.
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    intrs[1].width = 320;   // camera 0 is 640x480
    auto extrs = MakeSideCameras({0, 90, 180, 270});
    StitchEngine engine;
    EXPECT_THROW(engine.Configure(intrs, extrs, BowlParams::Default()),
                 std::invalid_argument);
}

TEST(StitchEngineTest, ConfigureN1_N8_Success) {
    for (int n = 1; n <= 8; ++n) {
        auto intrs = std::vector<CameraIntrinsics>(n, MakeIntrinsics());
        std::vector<double> azimuths;
        for (int i = 0; i < n; ++i) azimuths.push_back(i * 360.0 / n);
        auto extrs = MakeSideCameras(azimuths);
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
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 90, 180, 270});
    StitchEngine engine;
    engine.Configure(intrs, extrs, BowlParams::Default());
    EXPECT_TRUE(engine.IsConfigured());
}

TEST(StitchEngineTest, GetNumCameras) {
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 90, 180, 270});
    StitchEngine engine;
    engine.Configure(intrs, extrs, BowlParams::Default());
    EXPECT_EQ(4, engine.GetNumCameras());
}

TEST(StitchEngineTest, RenderFrameBeforeConfigureThrows) {
    StitchEngine engine;
    EXPECT_THROW(engine.RenderFrame({}), std::runtime_error);
}

TEST(StitchEngineTest, RenderFrameWrongImageCount) {
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 90, 180, 270});
    StitchEngine engine;
    engine.Configure(intrs, extrs, BowlParams::Default());

    auto frames = MakeColourFrames(3);   // 3 images for a 4-camera engine
    EXPECT_THROW(engine.RenderFrame(frames), std::invalid_argument);
}

TEST(StitchEngineTest, Configure_BowlParamsCarLengthWidth) {
    // Truck-sized bowl params must configure without throwing.
    BowlParams bowl = BowlParams::Default();
    bowl.car_length = 18.0;
    bowl.car_width = 2.5;
    bowl.r_outter = 25.0;
    bowl.step = 1.0;   // coarse grid keeps the test fast

    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 90, 180, 270});
    StitchEngine engine;
    EXPECT_NO_THROW(engine.Configure(intrs, extrs, bowl));
    EXPECT_TRUE(engine.IsConfigured());
}

TEST(StitchEngineTest, ConfigureStepZeroThrows) {
    // step <= 0 makes the grid degenerate (rows = r_outter/step blows up or
    // divides by zero) — reject it with a clear error instead of UB.
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 90, 180, 270});
    BowlParams bowl = BowlParams::Default();
    bowl.step = 0.0;
    StitchEngine engine;
    EXPECT_THROW(engine.Configure(intrs, extrs, bowl), std::invalid_argument);
}

TEST(StitchEngineTest, ConfigureNonPositiveRoutterThrows) {
    // r_outter <= 0 would produce an empty/negative-extent bowl.
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 90, 180, 270});
    BowlParams bowl = BowlParams::Default();
    bowl.r_outter = 0.0;
    StitchEngine engine;
    EXPECT_THROW(engine.Configure(intrs, extrs, bowl), std::invalid_argument);
}

TEST(StitchEngineTest, ConfigureNegativeKSlopThrows) {
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 90, 180, 270});
    BowlParams bowl = BowlParams::Default();
    bowl.k_slop = -0.1;
    StitchEngine engine;
    EXPECT_THROW(engine.Configure(intrs, extrs, bowl), std::invalid_argument);
}

TEST(StitchEngineTest, ConfigureNegativeZExpThrows) {
    // z_exp < 0 makes the bowl rise formula (z = k_slop * rd^z_exp) blow up
    // for rd < 1 — reject it with a clear error instead of emitting NaN/inf
    // vertex positions into the mesh.
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 90, 180, 270});
    BowlParams bowl = BowlParams::Default();
    bowl.z_exp = -1.0;
    StitchEngine engine;
    EXPECT_THROW(engine.Configure(intrs, extrs, bowl), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Task 8: StitchEngine GL end-to-end tests (gated).
// ---------------------------------------------------------------------------
TEST_F(OSGGLTest, StitchEngineGL_RenderFrame_Returns5Views) {
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeLookAtCameras(4);
    StitchEngine engine;
    engine.Configure(intrs, extrs, BowlParams::Default());

    auto views = engine.RenderFrame(MakeColourFrames(4));
    ASSERT_EQ(5u, views.size());
    EXPECT_NE(views.end(), views.find("topdown"));
    EXPECT_NE(views.end(), views.find("3d_front"));
    EXPECT_NE(views.end(), views.find("3d_rear"));
    EXPECT_NE(views.end(), views.find("3d_left"));
    EXPECT_NE(views.end(), views.find("3d_right"));
}

TEST_F(OSGGLTest, StitchEngineGL_RenderFrame_OutputSize) {
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeLookAtCameras(4);
    StitchEngine engine;
    engine.Configure(intrs, extrs, BowlParams::Default());

    auto views = engine.RenderFrame(MakeColourFrames(4));
    ASSERT_EQ(5u, views.size());
    for (const auto& kv : views) {
        EXPECT_EQ(960, kv.second.cols) << kv.first;
        EXPECT_EQ(640, kv.second.rows) << kv.first;
        EXPECT_EQ(CV_8UC3, kv.second.type()) << kv.first;
    }
}

TEST_F(OSGGLTest, StitchEngineGL_RenderFrame_NonBlackOutput) {
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeLookAtCameras(4);
    StitchEngine engine;
    engine.Configure(intrs, extrs, BowlParams::Default());

    auto views = engine.RenderFrame(MakeColourFrames(4));
    ASSERT_EQ(5u, views.size());
    // The bowl ground visible to the look-at cameras must render camera
    // colours in EVERY view — not just the clear colour / black. Background
    // clear colour is RGB(30,36,50), a low-saturation colour, so vivid
    // (mx-mn>100) pixels prove the texture upload + mapping shader ran.
    for (const auto& kv : views) {
        EXPECT_GT(CountVividPixels(kv.second), 0) << kv.first;
    }
}

TEST_F(OSGGLTest, StitchEngineGL_RenderFrame_N8Cameras) {
    auto intrs = std::vector<CameraIntrinsics>(8, MakeIntrinsics());
    auto extrs = MakeLookAtCameras(8);
    StitchEngine engine;
    engine.Configure(intrs, extrs, BowlParams::Default());

    auto views = engine.RenderFrame(MakeColourFrames(8));
    ASSERT_EQ(5u, views.size());
    int total_vivid = 0;
    for (const auto& kv : views) total_vivid += CountVividPixels(kv.second);
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
    auto intrs = std::vector<CameraIntrinsics>{MakeIntrinsics()};
    auto extrs =
        std::vector<CameraExtrinsics>{MakeLookAtCamera(0.0, 4.0, 2.0, 0.0)};
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
    auto intrs = std::vector<CameraIntrinsics>{MakeIntrinsics()};
    auto extrs =
        std::vector<CameraExtrinsics>{MakeLookAtCamera(0.0, 4.0, 2.0, 0.0)};
    StitchEngine engine;
    engine.Configure(intrs, extrs, BowlParams::Default());

    auto views = engine.RenderFrame({MakeBandFrame()});
    auto it = views.find("3d_front");
    ASSERT_NE(views.end(), it);
    const cv::Mat& front = it->second;

    // The far rim (imaged near the camera's image TOP, band 0) renders at the
    // top of the 3d_front output; the near flat ground (camera image BOTTOM,
    // band 5) renders at the bottom. A mirrored renderer swaps these. Rows
    // 0..80 are the far rim; rows 480..560 are the near-ground band 5.
    EXPECT_EQ(0, DominantBand(front, 0, 30))
        << "3d_front output top rows must sample camera image band 0";
    EXPECT_EQ(5, DominantBand(front, 480, 560))
        << "3d_front output bottom rows must sample camera image band 5";
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
    const auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());

    // 1) Realistic AVM ring: 4 look-at cameras at radius 4m, height 2m, each
    //    aimed at the bowl centre. Sky/ground split frames.
    {
        auto extrs = MakeLookAtCameras(4);
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
        auto extrs = MakeLookAtCameras(4);
        StitchEngine engine;
        engine.Configure(intrs, extrs, BowlParams::Default());
        auto views = engine.RenderFrame(MakeColourFrames(4));
        ASSERT_EQ(5u, views.size());
        EXPECT_EQ(5, DumpViews(views, dir, "lookat_colour"));
    }

    // 3) Original side cameras (identity rotation, radius 3m, height 1m) with
    //    the same sky/ground frames, for comparison.
    {
        auto extrs = MakeSideCameras({0, 90, 180, 270});
        StitchEngine engine;
        engine.Configure(intrs, extrs, BowlParams::Default());
        auto views = engine.RenderFrame(MakeSkyGroundFrames(4));
        ASSERT_EQ(5u, views.size());
        EXPECT_EQ(5, DumpViews(views, dir, "side_skyground"));
        std::cout << "side_skyground views:\n";
        for (const auto& kv : views) PrintSkyGroundStats(kv.second, kv.first);
    }
}


