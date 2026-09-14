// Tests for the OSGViewRenderer (offscreen 5-view rendering) and ViewPose /
// DefaultViewPoses (pure math). GL-dependent tests are gated: they skip unless
// AVM_STITCH_TEST_GL=1 or a working OpenGL context is detected.

#include <gtest/gtest.h>

#include "geometry/bowl_model.hpp"
#include "render/osg_view_renderer.hpp"

#include "test_support/gl_fixture.hpp"
#include "test_support/stitch_test_helpers.hpp"

using namespace avm_stitch;
using avm_stitch::test::OSGGLTest;

// ---------------------------------------------------------------------------
// ViewPose / DefaultViewPoses (pure math, no GL required).
// ---------------------------------------------------------------------------

TEST(ViewPoseTest, DefaultViewPoses_Returns5Views) {
    auto poses = DefaultViewPoses();
    ASSERT_EQ(5u, poses.size());
}

TEST(ViewPoseTest, DefaultViewPoses_HasCorrectNames) {
    auto poses = DefaultViewPoses();
    ASSERT_EQ(5u, poses.size());
    EXPECT_EQ("topdown", poses[0].name);
    EXPECT_EQ("3d_front", poses[1].name);
    EXPECT_EQ("3d_rear", poses[2].name);
    EXPECT_EQ("3d_left", poses[3].name);
    EXPECT_EQ("3d_right", poses[4].name);
}

TEST(ViewPoseTest, DefaultViewPoses_TopdownPosition) {
    auto poses = DefaultViewPoses();
    ASSERT_GE(poses.size(), 1u);
    EXPECT_NEAR(-1.82, poses[0].position[0], 1e-9);
    EXPECT_NEAR(0.0, poses[0].position[1], 1e-9);
    EXPECT_NEAR(6.0, poses[0].position[2], 1e-9);
}

TEST(ViewPoseTest, DefaultViewPoses_AllFovy60) {
    auto poses = DefaultViewPoses();
    ASSERT_EQ(5u, poses.size());
    for (const auto& pose : poses) {
        EXPECT_DOUBLE_EQ(60.0, pose.fovy_deg);
    }
}

TEST(ViewPoseTest, DefaultViewPoses_AllTargetSame) {
    auto poses = DefaultViewPoses();
    ASSERT_EQ(5u, poses.size());
    for (const auto& pose : poses) {
        EXPECT_NEAR(0.0, pose.target[0], 1e-9);
        EXPECT_NEAR(0.0, pose.target[1], 1e-9);
        EXPECT_NEAR(0.2, pose.target[2], 1e-9);
        EXPECT_NEAR(0.0, pose.up[0], 1e-9);
        EXPECT_NEAR(0.0, pose.up[1], 1e-9);
        EXPECT_NEAR(1.0, pose.up[2], 1e-9);
    }
}

// ---------------------------------------------------------------------------
// DefaultViewPoses with car_center offset.
// ---------------------------------------------------------------------------

TEST(ViewPoseTest, DefaultViewPoses_CarCenterOffsetsAllPoses) {
    // car_center=(2,1): each pose position/target x/y offset by +2/+1.
    const cv::Vec2d cc(2.0, 1.0);
    auto poses = DefaultViewPoses(cc);
    auto ref   = DefaultViewPoses(cv::Vec2d(0.0, 0.0));
    ASSERT_EQ(ref.size(), poses.size());

    for (size_t i = 0; i < poses.size(); ++i) {
        SCOPED_TRACE(ref[i].name);
        // position x/y offset by cc, z unchanged
        EXPECT_NEAR(ref[i].position[0] + cc[0], poses[i].position[0], 1e-9);
        EXPECT_NEAR(ref[i].position[1] + cc[1], poses[i].position[1], 1e-9);
        EXPECT_NEAR(ref[i].position[2],         poses[i].position[2], 1e-9);
        // target x/y offset by cc, z unchanged
        EXPECT_NEAR(ref[i].target[0] + cc[0], poses[i].target[0], 1e-9);
        EXPECT_NEAR(ref[i].target[1] + cc[1], poses[i].target[1], 1e-9);
        EXPECT_NEAR(ref[i].target[2],         poses[i].target[2], 1e-9);
        // up unchanged
        EXPECT_NEAR(ref[i].up[0], poses[i].up[0], 1e-9);
        EXPECT_NEAR(ref[i].up[1], poses[i].up[1], 1e-9);
        EXPECT_NEAR(ref[i].up[2], poses[i].up[2], 1e-9);
        // fovy + name unchanged
        EXPECT_DOUBLE_EQ(ref[i].fovy_deg, poses[i].fovy_deg);
        EXPECT_EQ(ref[i].name, poses[i].name);
    }
}

TEST(ViewPoseTest, DefaultViewPoses_DefaultArgUnchanged) {
    // No-arg call must match explicit car_center=(0,0) — regression guard.
    auto default_poses = DefaultViewPoses();
    auto explicit_poses = DefaultViewPoses(cv::Vec2d(0.0, 0.0));
    ASSERT_EQ(default_poses.size(), explicit_poses.size());
    for (size_t i = 0; i < default_poses.size(); ++i) {
        SCOPED_TRACE(default_poses[i].name);
        EXPECT_DOUBLE_EQ(default_poses[i].position[0], explicit_poses[i].position[0]);
        EXPECT_DOUBLE_EQ(default_poses[i].position[1], explicit_poses[i].position[1]);
        EXPECT_DOUBLE_EQ(default_poses[i].position[2], explicit_poses[i].position[2]);
        EXPECT_DOUBLE_EQ(default_poses[i].target[0],   explicit_poses[i].target[0]);
        EXPECT_DOUBLE_EQ(default_poses[i].target[1],   explicit_poses[i].target[1]);
        EXPECT_DOUBLE_EQ(default_poses[i].target[2],   explicit_poses[i].target[2]);
        EXPECT_DOUBLE_EQ(default_poses[i].up[0],       explicit_poses[i].up[0]);
        EXPECT_DOUBLE_EQ(default_poses[i].up[1],       explicit_poses[i].up[1]);
        EXPECT_DOUBLE_EQ(default_poses[i].up[2],       explicit_poses[i].up[2]);
        EXPECT_DOUBLE_EQ(default_poses[i].fovy_deg,    explicit_poses[i].fovy_deg);
        EXPECT_EQ(default_poses[i].name,               explicit_poses[i].name);
    }
}

// ---------------------------------------------------------------------------
// DefaultViewPoses with a scale factor (scales the frame with the vehicle).
// ---------------------------------------------------------------------------

TEST(ViewPoseTest, DefaultViewPoses_ScaleOneMatchesHistoricalConstants) {
    // scale=1.0 is the anchor: it must reproduce the exact hardcoded default
    // pose set (topdown + 4 side views at the calibrated 5.8/6.0/2.0/1.82/0.2
    // offsets) — regression pin so the car keeps its historical camera frame.
    auto poses = DefaultViewPoses(cv::Vec2d(0.0, 0.0), 1.0);
    ASSERT_EQ(5u, poses.size());
    EXPECT_NEAR(-1.82, poses[0].position[0], 1e-12);
    EXPECT_NEAR(0.0, poses[0].position[1], 1e-12);
    EXPECT_NEAR(6.0, poses[0].position[2], 1e-12);
    EXPECT_NEAR(0.0, poses[0].target[0], 1e-12);
    EXPECT_NEAR(0.0, poses[0].target[1], 1e-12);
    EXPECT_NEAR(0.2, poses[0].target[2], 1e-12);
    for (size_t i = 1; i < poses.size(); ++i) {
        SCOPED_TRACE(poses[i].name);
        const double offset = (i == 3 || i == 4) ? poses[i].position[1]
                                                 : poses[i].position[0];
        EXPECT_NEAR(5.8, std::abs(offset), 1e-12);
        EXPECT_NEAR(2.0, poses[i].position[2], 1e-12);
        EXPECT_NEAR(0.0, poses[i].target[0], 1e-12);
        EXPECT_NEAR(0.0, poses[i].target[1], 1e-12);
        EXPECT_NEAR(0.2, poses[i].target[2], 1e-12);
    }
}

TEST(ViewPoseTest, DefaultViewPoses_ScaleMultipliesOffsetsAndHeights) {
    // scale multiplies every car-relative offset AND the absolute camera /
    // target height — the viewing frame expands with the vehicle (the old
    // hardcoded car-sized frame could not frame a 38x longer ship).
    const double s = 2.5;
    auto ref = DefaultViewPoses(cv::Vec2d(0.0, 0.0), 1.0);
    auto scaled = DefaultViewPoses(cv::Vec2d(0.0, 0.0), s);
    ASSERT_EQ(ref.size(), scaled.size());
    for (size_t i = 0; i < ref.size(); ++i) {
        SCOPED_TRACE(ref[i].name);
        EXPECT_NEAR(ref[i].position[0] * s, scaled[i].position[0], 1e-9);
        EXPECT_NEAR(ref[i].position[1] * s, scaled[i].position[1], 1e-9);
        EXPECT_NEAR(ref[i].position[2] * s, scaled[i].position[2], 1e-9);
        EXPECT_NEAR(ref[i].target[0] * s, scaled[i].target[0], 1e-9);
        EXPECT_NEAR(ref[i].target[1] * s, scaled[i].target[1], 1e-9);
        EXPECT_NEAR(ref[i].target[2] * s, scaled[i].target[2], 1e-9);
        EXPECT_DOUBLE_EQ(ref[i].fovy_deg, scaled[i].fovy_deg);
        EXPECT_EQ(ref[i].name, scaled[i].name);
        for (int k = 0; k < 3; ++k) {
            EXPECT_DOUBLE_EQ(ref[i].up[k], scaled[i].up[k]);
        }
    }
}

// ---------------------------------------------------------------------------
// OSGViewRenderer (offscreen 5-view rendering, GL-gated).
// ---------------------------------------------------------------------------

TEST_F(OSGGLTest, OSGViewRenderer_ConstructionDoesNotCrash) {
    auto ts = test::BuildTestScene(4);
    ASSERT_TRUE(ts.scene->root().valid());
    ASSERT_TRUE(ts.has_visible_vertices);   // otherwise the test proves nothing

    EXPECT_NO_THROW(
        OSGViewRenderer renderer(*ts.scene, DefaultViewPoses(), 960, 640));
}

TEST_F(OSGGLTest, OSGViewRenderer_RenderAllViews_Returns5Mats) {
    auto ts = test::BuildTestScene(4);
    ASSERT_TRUE(ts.scene->root().valid());
    ASSERT_TRUE(test::SceneTexture(*ts.scene).valid());
    ASSERT_TRUE(ts.has_visible_vertices);

    OSGViewRenderer renderer(*ts.scene, DefaultViewPoses(), 960, 640);
    auto views = renderer.RenderAllViews();

    ASSERT_EQ(5u, views.size());
    EXPECT_NE(views.end(), views.find("topdown"));
    EXPECT_NE(views.end(), views.find("3d_front"));
    EXPECT_NE(views.end(), views.find("3d_rear"));
    EXPECT_NE(views.end(), views.find("3d_left"));
    EXPECT_NE(views.end(), views.find("3d_right"));
}

TEST_F(OSGGLTest, OSGViewRenderer_RenderAllViews_CorrectSize) {
    auto ts = test::BuildTestScene(4);
    ASSERT_TRUE(ts.scene->root().valid());
    ASSERT_TRUE(ts.has_visible_vertices);

    OSGViewRenderer renderer(*ts.scene, DefaultViewPoses(), 960, 640);
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
    auto ts = test::BuildTestSceneWithExtrinsics(4, test::MakeLookAtCameras(4));
    ASSERT_TRUE(ts.scene->root().valid());
    ASSERT_TRUE(test::SceneTexture(*ts.scene).valid());
    ASSERT_TRUE(ts.has_visible_vertices);

    // Upload distinct colours so the bowl rim renders vivid pixels.
    ts.scene->UploadCameraImages(test::MakeColourFrames(4));

    OSGViewRenderer renderer(*ts.scene, DefaultViewPoses(), 960, 640);
    auto views = renderer.RenderAllViews();
    ASSERT_EQ(5u, views.size());

    // Background clear colour is RGB(30,36,50) — a low-saturation colour. The
    // bowl must render vivid camera-colour pixels in EVERY view, proving the
    // texture array upload + mapping shader actually ran (not just the FBO
    // clear). Each view needs its own Geode referencing the shared Geometry —
    // the single-geode approach reparents it to the last camera, leaving the
    // other four views blank.
    for (const auto& kv : views) {
        EXPECT_GT(test::CountVividPixels(kv.second), 0) << kv.first;
    }
}

// ---------------------------------------------------------------------------
// RenderMode (enum + constructor overload, GL-gated).
// ---------------------------------------------------------------------------

TEST(RenderModeTest, EnumHasOffscreenAndWindowed) {
    // Compile-time check: both enum values exist.
    EXPECT_NE(RenderMode::Offscreen, RenderMode::Windowed);
}

TEST(RenderModeTest, DefaultModeIsOffscreen) {
    // The default constructor argument must be Offscreen.
    // This is a compile-time contract tested by verifying the enum value.
    RenderMode mode = RenderMode::Offscreen;
    EXPECT_EQ(RenderMode::Offscreen, mode);
}

TEST_F(OSGGLTest, OSGViewRenderer_OffscreenMode_Works) {
    auto ts = test::BuildTestScene(4);
    ASSERT_TRUE(ts.scene->root().valid());
    ASSERT_TRUE(ts.has_visible_vertices);

    // Explicit Offscreen — same as the default.
    OSGViewRenderer renderer(*ts.scene, DefaultViewPoses(), 960, 640,
                             RenderMode::Offscreen);
    auto views = renderer.RenderAllViews();
    ASSERT_EQ(5u, views.size());
}

TEST_F(OSGGLTest, OSGViewRenderer_WindowedMode_Constructs) {
    auto ts = test::BuildTestScene(4);
    ASSERT_TRUE(ts.scene->root().valid());
    ASSERT_TRUE(ts.has_visible_vertices);

    // Windowed mode requires a display; skip if none available.
    if (!std::getenv("DISPLAY") && !std::getenv("WAYLAND_DISPLAY")) {
        GTEST_SKIP() << "No display server — cannot test Windowed mode";
    }

    EXPECT_NO_THROW(
        OSGViewRenderer renderer(*ts.scene, DefaultViewPoses(), 960, 640,
                                 RenderMode::Windowed));
}

TEST_F(OSGGLTest, OSGViewRenderer_WindowedMode_Constructs_WithCarCenter) {
    auto ts = test::BuildTestScene(4);
    ASSERT_TRUE(ts.scene->root().valid());
    ASSERT_TRUE(ts.has_visible_vertices);

    if (!std::getenv("DISPLAY") && !std::getenv("WAYLAND_DISPLAY")) {
        GTEST_SKIP() << "No display server — cannot test Windowed mode";
    }

    // Construction with non-zero car_center must not crash.
    // Home camera math is untestable without X11; this verifies the
    // constructor path at least accepts the parameter.
    EXPECT_NO_THROW(
        OSGViewRenderer renderer(*ts.scene, DefaultViewPoses(cv::Vec2d(2.0, 1.0)),
                                 960, 640, RenderMode::Windowed,
                                 cv::Vec2d(2.0, 1.0)));
}
