
#include <gtest/gtest.h>

#include "bowl_model/bowl_model.hpp"
#include "render/osg_view_renderer.hpp"

#include "test_support/gl_fixture.hpp"
#include "test_support/stitch_test_helpers.hpp"

using namespace avm_stitch;
using avm_stitch::test::OSGGLTest;


TEST(ViewPoseTest, DefaultViewPoses_Returns4Views) {
    auto poses = DefaultViewPoses();
    ASSERT_EQ(4u, poses.size());
}

TEST(ViewPoseTest, DefaultViewPoses_HasCorrectNames) {
    auto poses = DefaultViewPoses();
    ASSERT_EQ(4u, poses.size());
    EXPECT_EQ("3d_front", poses[0].name);
    EXPECT_EQ("3d_rear", poses[1].name);
    EXPECT_EQ("3d_left", poses[2].name);
    EXPECT_EQ("3d_right", poses[3].name);
        for (const auto& pose : poses) {
        EXPECT_NE("topdown", pose.name);
    }
}

TEST(ViewPoseTest, DefaultViewPoses_AllFovy60) {
    auto poses = DefaultViewPoses();
    ASSERT_EQ(4u, poses.size());
    for (const auto& pose : poses) {
        EXPECT_DOUBLE_EQ(60.0, pose.fovy_deg);
    }
}

TEST(ViewPoseTest, DefaultViewPoses_AllTargetSame) {
    auto poses = DefaultViewPoses();
    ASSERT_EQ(4u, poses.size());
    for (const auto& pose : poses) {
        EXPECT_NEAR(0.0, pose.target[0], 1e-9);
        EXPECT_NEAR(0.0, pose.target[1], 1e-9);
        EXPECT_NEAR(0.2, pose.target[2], 1e-9);
        EXPECT_NEAR(0.0, pose.up[0], 1e-9);
        EXPECT_NEAR(0.0, pose.up[1], 1e-9);
        EXPECT_NEAR(1.0, pose.up[2], 1e-9);
    }
}


TEST(ViewPoseTest, DefaultViewPoses_CarCenterOffsetsAllPoses) {
        const cv::Vec2d cc(2.0, 1.0);
    auto poses = DefaultViewPoses(cc);
    auto ref   = DefaultViewPoses(cv::Vec2d(0.0, 0.0));
    ASSERT_EQ(ref.size(), poses.size());

    for (size_t i = 0; i < poses.size(); ++i) {
        SCOPED_TRACE(ref[i].name);
                EXPECT_NEAR(ref[i].position[0] + cc[0], poses[i].position[0], 1e-9);
        EXPECT_NEAR(ref[i].position[1] + cc[1], poses[i].position[1], 1e-9);
        EXPECT_NEAR(ref[i].position[2],         poses[i].position[2], 1e-9);
                EXPECT_NEAR(ref[i].target[0] + cc[0], poses[i].target[0], 1e-9);
        EXPECT_NEAR(ref[i].target[1] + cc[1], poses[i].target[1], 1e-9);
        EXPECT_NEAR(ref[i].target[2],         poses[i].target[2], 1e-9);
                EXPECT_NEAR(ref[i].up[0], poses[i].up[0], 1e-9);
        EXPECT_NEAR(ref[i].up[1], poses[i].up[1], 1e-9);
        EXPECT_NEAR(ref[i].up[2], poses[i].up[2], 1e-9);
                EXPECT_DOUBLE_EQ(ref[i].fovy_deg, poses[i].fovy_deg);
        EXPECT_EQ(ref[i].name, poses[i].name);
    }
}

TEST(ViewPoseTest, DefaultViewPoses_DefaultArgUnchanged) {
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


TEST(ViewPoseTest, DefaultViewPoses_ScaleOneMatchesHistoricalConstants) {
                auto poses = DefaultViewPoses(cv::Vec2d(0.0, 0.0), 1.0);
    ASSERT_EQ(4u, poses.size());
    EXPECT_NEAR(5.8, std::abs(poses[0].position[0]), 1e-12);
    EXPECT_NEAR(0.0, poses[0].position[1], 1e-12);
    EXPECT_NEAR(2.0, poses[0].position[2], 1e-12);
    EXPECT_NEAR(0.0, poses[0].target[0], 1e-12);
    EXPECT_NEAR(0.0, poses[0].target[1], 1e-12);
    EXPECT_NEAR(0.2, poses[0].target[2], 1e-12);
    for (size_t i = 0; i < poses.size(); ++i) {
        SCOPED_TRACE(poses[i].name);
        const double offset = (i == 2 || i == 3) ? poses[i].position[1]
                                                 : poses[i].position[0];
        EXPECT_NEAR(5.8, std::abs(offset), 1e-12);
        EXPECT_NEAR(2.0, poses[i].position[2], 1e-12);
        EXPECT_NEAR(0.0, poses[i].target[0], 1e-12);
        EXPECT_NEAR(0.0, poses[i].target[1], 1e-12);
        EXPECT_NEAR(0.2, poses[i].target[2], 1e-12);
    }
}

TEST(ViewPoseTest, DefaultViewPoses_ScaleMultipliesOffsetsAndHeights) {
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


namespace {

double MaxEyeToTarget(double scale) {
    double max_eye = 0.0;
    for (const auto& p : DefaultViewPoses(cv::Vec2d(0.0, 0.0), scale)) {
        max_eye = std::max(max_eye, cv::norm(p.position - p.target));
    }
    return max_eye;
}

}
TEST(ClipPlanesTest, ComputeClipPlanes_CarScale_CoversBowl) {
                const double scene_radius = kCalibratedAnchorRoutter;
    const double max_eye = MaxEyeToTarget(1.0);
    ClipPlanes cp = ComputeClipPlanes(scene_radius, max_eye);

    EXPECT_NEAR(0.1, cp.near, 1e-3);
    EXPECT_NEAR(100.0, cp.far, 1e-2);

        EXPECT_GT(cp.far, max_eye + 2.0 * scene_radius);
            EXPECT_LT(cp.near, 1.0);
    EXPECT_LT(cp.near, cp.far);
        EXPECT_GE(cp.near / cp.far, 5.0e-4);
    EXPECT_LE(cp.near / cp.far, 1.0e-2);
}

TEST(ClipPlanesTest, ComputeClipPlanes_ShipScale_CoversBowl) {
                const double scene_radius = 476.0;
    const double max_eye = MaxEyeToTarget(scene_radius / kCalibratedAnchorRoutter);
    ClipPlanes cp = ComputeClipPlanes(scene_radius, max_eye);

    EXPECT_GT(max_eye, 200.0);       EXPECT_GT(cp.far, 1000.0);       EXPECT_GT(cp.far, max_eye + scene_radius);
    EXPECT_LT(cp.near, 10.0);        EXPECT_GT(cp.near, 0.1);         EXPECT_LT(cp.near, cp.far);
        EXPECT_GE(cp.near / cp.far, 5.0e-4);
    EXPECT_LE(cp.near / cp.far, 1.0e-2);
}

TEST(ClipPlanesTest, ComputeClipPlanes_ScaleInvariant) {
            const double K = 476.0 / kCalibratedAnchorRoutter;
    const ClipPlanes car  = ComputeClipPlanes(kCalibratedAnchorRoutter,
                                              MaxEyeToTarget(1.0));
    const ClipPlanes ship = ComputeClipPlanes(kCalibratedAnchorRoutter * K,
                                              MaxEyeToTarget(K));

    EXPECT_NEAR(car.near * K, ship.near, std::abs(car.near * K) * 1e-6);
    EXPECT_NEAR(car.far * K, ship.far, std::abs(car.far * K) * 1e-6);
}


TEST_F(OSGGLTest, OSGViewRenderer_ConstructionDoesNotCrash) {
    auto ts = test::BuildTestScene(4);
    ASSERT_TRUE(ts.scene->root().valid());
    ASSERT_TRUE(ts.has_visible_vertices);
    EXPECT_NO_THROW(
        OSGViewRenderer renderer(*ts.scene, kCalibratedAnchorRoutter, DefaultViewPoses(), 960, 640));
}

TEST_F(OSGGLTest, OSGViewRenderer_RenderAllViews_Returns4Mats) {
    auto ts = test::BuildTestScene(4);
    ASSERT_TRUE(ts.scene->root().valid());
    ASSERT_TRUE(test::SceneTexture(*ts.scene).valid());
    ASSERT_TRUE(ts.has_visible_vertices);

    OSGViewRenderer renderer(*ts.scene, kCalibratedAnchorRoutter, DefaultViewPoses(), 960, 640);
    auto views = renderer.RenderAllViews();

    ASSERT_EQ(4u, views.size());
    EXPECT_NE(views.end(), views.find("3d_front"));
    EXPECT_NE(views.end(), views.find("3d_rear"));
    EXPECT_NE(views.end(), views.find("3d_left"));
    EXPECT_NE(views.end(), views.find("3d_right"));
    EXPECT_EQ(views.end(), views.find("topdown"));
}

TEST_F(OSGGLTest, OSGViewRenderer_RenderAllViews_CorrectSize) {
    auto ts = test::BuildTestScene(4);
    ASSERT_TRUE(ts.scene->root().valid());
    ASSERT_TRUE(ts.has_visible_vertices);

    OSGViewRenderer renderer(*ts.scene, kCalibratedAnchorRoutter, DefaultViewPoses(), 960, 640);
    auto views = renderer.RenderAllViews();
    ASSERT_EQ(4u, views.size());

    for (const auto& kv : views) {
        EXPECT_EQ(960, kv.second.cols) << kv.first;
        EXPECT_EQ(640, kv.second.rows) << kv.first;
        EXPECT_EQ(CV_8UC3, kv.second.type()) << kv.first;
    }
}

TEST_F(OSGGLTest, OSGViewRenderer_RenderAllViews_DrawsSceneNotJustBackground) {
                auto ts = test::BuildTestSceneWithExtrinsics(4, test::MakeLookAtCameras(4));
    ASSERT_TRUE(ts.scene->root().valid());
    ASSERT_TRUE(test::SceneTexture(*ts.scene).valid());
    ASSERT_TRUE(ts.has_visible_vertices);

        ts.scene->UploadCameraImages(test::MakeColourFrames(4));

    OSGViewRenderer renderer(*ts.scene, kCalibratedAnchorRoutter, DefaultViewPoses(), 960, 640);
    auto views = renderer.RenderAllViews();
    ASSERT_EQ(4u, views.size());

                            for (const auto& kv : views) {
        EXPECT_GT(test::CountVividPixels(kv.second), 0) << kv.first;
    }
}

TEST_F(OSGGLTest, OSGViewRenderer_RenderAllViews_ShipScale_NotAllBackground) {
                        const BowlParams bp = DeriveBowlParams(211.6, 50.0, 50.0);
    ASSERT_GT(bp.r_outter(), 450.0);
    std::vector<KbParams> intrs(8, test::MakeIntrinsics());
    std::vector<CameraExtrinsics> extrs;
    for (int i = 0; i < 8; ++i) {
        extrs.push_back(test::MakeLookAtCamera(i * 45.0, 220.0, 80.0, 0.0));
    }
    BowlModel bowl(bp, test::WrapKbCameras(intrs, extrs));
    ASSERT_TRUE(bowl.HasVisibleVertices());
    StitchScene scene(bowl, intrs[0].width, intrs[0].height);
    ASSERT_TRUE(scene.root().valid());
    scene.UploadCameraImages(test::MakeColourFrames(8));

    const double scale = bp.r_outter() / kCalibratedAnchorRoutter;
    OSGViewRenderer renderer(scene, bp.r_outter(),
                             DefaultViewPoses(bp.car_center(), scale),
                             960, 640, RenderMode::Offscreen, bp.car_center());
    auto views = renderer.RenderAllViews();
    ASSERT_EQ(4u, views.size());

    for (const auto& kv : views) {
        EXPECT_GT(test::CountVividPixels(kv.second), 1000) << kv.first;
    }
}

//

TEST_F(OSGGLTest, OSGViewRenderer_OffscreenMode_Works) {
    auto ts = test::BuildTestScene(4);
    ASSERT_TRUE(ts.scene->root().valid());
    ASSERT_TRUE(ts.has_visible_vertices);

        OSGViewRenderer renderer(*ts.scene, kCalibratedAnchorRoutter, DefaultViewPoses(), 960, 640,
                             RenderMode::Offscreen);
    auto views = renderer.RenderAllViews();
    ASSERT_EQ(4u, views.size());
}

TEST_F(OSGGLTest, OSGViewRenderer_WindowedMode_Constructs) {
    auto ts = test::BuildTestScene(4);
    ASSERT_TRUE(ts.scene->root().valid());
    ASSERT_TRUE(ts.has_visible_vertices);

        if (!std::getenv("DISPLAY") && !std::getenv("WAYLAND_DISPLAY")) {
        GTEST_SKIP() << "No display server — cannot test Windowed mode";
    }

    EXPECT_NO_THROW(
        OSGViewRenderer renderer(*ts.scene, kCalibratedAnchorRoutter, DefaultViewPoses(), 960, 640,
                                 RenderMode::Windowed));
}

TEST_F(OSGGLTest, OSGViewRenderer_WindowedMode_Constructs_WithCarCenter) {
    auto ts = test::BuildTestScene(4);
    ASSERT_TRUE(ts.scene->root().valid());
    ASSERT_TRUE(ts.has_visible_vertices);

    if (!std::getenv("DISPLAY") && !std::getenv("WAYLAND_DISPLAY")) {
        GTEST_SKIP() << "No display server — cannot test Windowed mode";
    }

                EXPECT_NO_THROW(
        OSGViewRenderer renderer(*ts.scene, kCalibratedAnchorRoutter,
                                 DefaultViewPoses(cv::Vec2d(2.0, 1.0)),
                                 960, 640, RenderMode::Windowed,
                                 cv::Vec2d(2.0, 1.0)));
}
