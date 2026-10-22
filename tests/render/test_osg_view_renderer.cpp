
#include <gtest/gtest.h>

#include "bowl_model/bowl_model.hpp"
#include "render/osg_view_renderer.hpp"

#include "test_support/gl_fixture.hpp"
#include "test_support/stitch_test_helpers.hpp"

using namespace avm_stitch;
using avm_stitch::test::OSGGLTest;

namespace {
BowlParams BowlAtScale(double scale, cv::Vec2d car_center = cv::Vec2d(0.0, 0.0)) {
    return BowlParams(2.0, 1.0, 0.5, 0.1, kCalibratedAnchorOuterRadius * scale,
                      3.2, 1.0, 15.0, car_center);
}
}

TEST(ViewPoseTest, DefaultViewPoses_Returns4Views) {
    auto poses = BowlAtScale(1.0).DefaultViewPoses();
    ASSERT_EQ(4u, poses.size());
}

TEST(ViewPoseTest, DefaultViewPoses_HasCorrectNames) {
    auto poses = BowlAtScale(1.0).DefaultViewPoses();
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
    auto poses = BowlAtScale(1.0).DefaultViewPoses();
    ASSERT_EQ(4u, poses.size());
    for (const auto& pose : poses) {
        EXPECT_DOUBLE_EQ(60.0, pose.fovy_deg);
    }
}

TEST(ViewPoseTest, DefaultViewPoses_AllTargetSame) {
    auto poses = BowlAtScale(1.0).DefaultViewPoses();
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
    auto poses = BowlAtScale(1.0, cc).DefaultViewPoses();
    auto ref = BowlAtScale(1.0).DefaultViewPoses();
    ASSERT_EQ(ref.size(), poses.size());

    for (size_t i = 0; i < poses.size(); ++i) {
        SCOPED_TRACE(ref[i].name);
        EXPECT_NEAR(ref[i].position[0] + cc[0], poses[i].position[0], 1e-9);
        EXPECT_NEAR(ref[i].position[1] + cc[1], poses[i].position[1], 1e-9);
        EXPECT_NEAR(ref[i].position[2], poses[i].position[2], 1e-9);
        EXPECT_NEAR(ref[i].target[0] + cc[0], poses[i].target[0], 1e-9);
        EXPECT_NEAR(ref[i].target[1] + cc[1], poses[i].target[1], 1e-9);
        EXPECT_NEAR(ref[i].target[2], poses[i].target[2], 1e-9);
        EXPECT_NEAR(ref[i].up[0], poses[i].up[0], 1e-9);
        EXPECT_NEAR(ref[i].up[1], poses[i].up[1], 1e-9);
        EXPECT_NEAR(ref[i].up[2], poses[i].up[2], 1e-9);
        EXPECT_DOUBLE_EQ(ref[i].fovy_deg, poses[i].fovy_deg);
        EXPECT_EQ(ref[i].name, poses[i].name);
    }
}

TEST(ViewPoseTest, DefaultViewPoses_ScaleOneMatchesHistoricalConstants) {
    auto poses = BowlAtScale(1.0).DefaultViewPoses();
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
    auto ref = BowlAtScale(1.0).DefaultViewPoses();
    auto scaled = BowlAtScale(s).DefaultViewPoses();
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

TEST(ViewPoseTest, HomeViewPose_BowlParamsAgreesWithHandBuiltScale) {
    const std::vector<BowlParams> shapes = {
        BowlAtScale(1.0),
        BowlAtScale(2.5),
    };
    for (const auto& bp : shapes) {
        SCOPED_TRACE(bp.OuterRadius());
        const double s = bp.OuterRadius() / kCalibratedAnchorOuterRadius;
        const auto via_bowl = bp.HomeViewPose();
        const ViewPose hand_built = {
            "", {10.0 * s, -6.0 * s, 6.0 * s}, {0.0, 0.0, 0.5 * s}, {0.0, 0.0, 1.0}, 60.0};
        EXPECT_DOUBLE_EQ(hand_built.position[0], via_bowl.position[0]);
        EXPECT_DOUBLE_EQ(hand_built.position[1], via_bowl.position[1]);
        EXPECT_DOUBLE_EQ(hand_built.position[2], via_bowl.position[2]);
        EXPECT_DOUBLE_EQ(hand_built.target[0], via_bowl.target[0]);
        EXPECT_DOUBLE_EQ(hand_built.target[1], via_bowl.target[1]);
        EXPECT_DOUBLE_EQ(hand_built.target[2], via_bowl.target[2]);
        EXPECT_DOUBLE_EQ(hand_built.up[0], via_bowl.up[0]);
        EXPECT_DOUBLE_EQ(hand_built.up[1], via_bowl.up[1]);
        EXPECT_DOUBLE_EQ(hand_built.up[2], via_bowl.up[2]);
        EXPECT_DOUBLE_EQ(hand_built.fovy_deg, via_bowl.fovy_deg);
        EXPECT_DOUBLE_EQ(12.0 * s, bp.HomeViewDistance());
    }
}

TEST(ViewPoseTest, HomeViewPose_CarCenterOffsetsPositionAndTarget) {
    const auto home = BowlAtScale(1.0, cv::Vec2d(2.0, 1.0)).HomeViewPose();
    EXPECT_NEAR(12.0, home.position[0], 1e-12);
    EXPECT_NEAR(-6.0, home.position[1], 1e-12);
    EXPECT_NEAR(5.0, home.position[2], 1e-12);
    EXPECT_NEAR(2.0, home.target[0], 1e-12);
    EXPECT_NEAR(0.0, home.target[1], 1e-12);
    EXPECT_NEAR(-0.5, home.target[2], 1e-12);
    EXPECT_NEAR(0.0, home.up[0], 1e-12);
    EXPECT_NEAR(0.0, home.up[1], 1e-12);
    EXPECT_NEAR(1.0, home.up[2], 1e-12);
    EXPECT_DOUBLE_EQ(60.0, home.fovy_deg);
}

TEST(ViewPoseTest, DefaultViewPoses_BowlParamsAgreesWithHandBuiltScale) {
    const std::vector<BowlParams> shapes = {
        BowlParams::ForCarSize(5.6, 2.2, 2.9),
        BowlParams::ForCarSize(12.6, 5.0, 6.3),
    };
    for (const auto& bp : shapes) {
        SCOPED_TRACE(bp.OuterRadius());
        const double s = bp.OuterRadius() / kCalibratedAnchorOuterRadius;
        auto via_bowl = bp.DefaultViewPoses();
        const auto hand_built = [&bp, s]() -> std::vector<ViewPose> {
            const double cx = bp.CarCenter()[0];
            const double cy = bp.CarCenter()[1];
            const double z_ground = 0.2 * s;
            return {
                {"3d_front", {cx + 5.8 * s, cy, 2.0 * s}, {cx, cy, z_ground}, {0, 0, 1}, 60.0},
                {"3d_rear", {cx - 5.8 * s, cy, 2.0 * s}, {cx, cy, z_ground}, {0, 0, 1}, 60.0},
                {"3d_left", {cx, cy + 5.8 * s, 2.0 * s}, {cx, cy, z_ground}, {0, 0, 1}, 60.0},
                {"3d_right", {cx, cy - 5.8 * s, 2.0 * s}, {cx, cy, z_ground}, {0, 0, 1}, 60.0},
            };
        }();
        ASSERT_EQ(hand_built.size(), via_bowl.size());
        for (size_t i = 0; i < via_bowl.size(); ++i) {
            SCOPED_TRACE(hand_built[i].name);
            EXPECT_EQ(hand_built[i].name, via_bowl[i].name);
            EXPECT_DOUBLE_EQ(hand_built[i].position[0], via_bowl[i].position[0]);
            EXPECT_DOUBLE_EQ(hand_built[i].position[1], via_bowl[i].position[1]);
            EXPECT_DOUBLE_EQ(hand_built[i].position[2], via_bowl[i].position[2]);
            EXPECT_DOUBLE_EQ(hand_built[i].target[0], via_bowl[i].target[0]);
            EXPECT_DOUBLE_EQ(hand_built[i].target[1], via_bowl[i].target[1]);
            EXPECT_DOUBLE_EQ(hand_built[i].target[2], via_bowl[i].target[2]);
            EXPECT_DOUBLE_EQ(hand_built[i].up[0], via_bowl[i].up[0]);
            EXPECT_DOUBLE_EQ(hand_built[i].up[1], via_bowl[i].up[1]);
            EXPECT_DOUBLE_EQ(hand_built[i].up[2], via_bowl[i].up[2]);
            EXPECT_DOUBLE_EQ(hand_built[i].fovy_deg, via_bowl[i].fovy_deg);
        }
    }
}

namespace {

double MaxEyeToTarget(double scale) {
    double max_eye = 0.0;
    for (const auto& p : BowlAtScale(scale).DefaultViewPoses()) {
        max_eye = std::max(max_eye, cv::norm(p.position - p.target));
    }
    return max_eye;
}

}
TEST(ClipPlanesTest, ClipPlanesFor_CarScale_CoversBowl) {
    const double scene_radius = kCalibratedAnchorOuterRadius;
    const double max_eye = MaxEyeToTarget(1.0);
    ClipPlanes cp = BowlParams(2.0, 1.0, 0.5, 0.1, scene_radius, 3.2, 1.0, 15.0)
                        .ClipPlanesFor(max_eye);

    EXPECT_NEAR(0.1, cp.near, 1e-3);
    EXPECT_NEAR(100.0, cp.far, 1e-2);

    EXPECT_GT(cp.far, max_eye + 2.0 * scene_radius);
    EXPECT_LT(cp.near, 1.0);
    EXPECT_LT(cp.near, cp.far);
    EXPECT_GE(cp.near / cp.far, 5.0e-4);
    EXPECT_LE(cp.near / cp.far, 1.0e-2);
}

TEST(ClipPlanesTest, ClipPlanesFor_ShipScale_CoversBowl) {
    const double scene_radius = 476.0;
    const double max_eye = MaxEyeToTarget(scene_radius / kCalibratedAnchorOuterRadius);
    ClipPlanes cp = BowlParams(2.0, 1.0, 0.5, 0.1, scene_radius, 3.2, 1.0, 15.0)
                        .ClipPlanesFor(max_eye);

    EXPECT_GT(max_eye, 200.0);
    EXPECT_GT(cp.far, 1000.0);
    EXPECT_GT(cp.far, max_eye + scene_radius);
    EXPECT_LT(cp.near, 10.0);
    EXPECT_GT(cp.near, 0.1);
    EXPECT_LT(cp.near, cp.far);
    EXPECT_GE(cp.near / cp.far, 5.0e-4);
    EXPECT_LE(cp.near / cp.far, 1.0e-2);
}

TEST(ClipPlanesTest, ClipPlanesFor_ScaleInvariant) {
    const double K = 476.0 / kCalibratedAnchorOuterRadius;
    const ClipPlanes car = BowlParams(2.0, 1.0, 0.5, 0.1, kCalibratedAnchorOuterRadius, 3.2, 1.0, 15.0)
                               .ClipPlanesFor(MaxEyeToTarget(1.0));
    const ClipPlanes ship = BowlParams(2.0, 1.0, 0.5, 0.1, kCalibratedAnchorOuterRadius * K, 3.2, 1.0, 15.0)
                                .ClipPlanesFor(MaxEyeToTarget(K));

    EXPECT_NEAR(car.near * K, ship.near, std::abs(car.near * K) * 1e-6);
    EXPECT_NEAR(car.far * K, ship.far, std::abs(car.far * K) * 1e-6);
}

TEST_F(OSGGLTest, OSGViewRenderer_ConstructionDoesNotCrash) {
    auto ts = test::BuildTestScene(4);
    ASSERT_TRUE(ts.scene->root().valid());
    ASSERT_TRUE(ts.has_visible_vertices);
    EXPECT_NO_THROW(
        OSGViewRenderer renderer(*ts.scene, BowlAtScale(1.0), 960, 640));
}

TEST_F(OSGGLTest, OSGViewRenderer_RenderAllViews_ShipScale_NotAllBackground) {
    const BowlParams bp = BowlParams::ForCarSize(211.6, 50.0, 50.0);
    ASSERT_GT(bp.OuterRadius(), 450.0);
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

    OSGViewRenderer renderer(scene, bp,
                             960, 640, RenderMode::Offscreen);
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

    OSGViewRenderer renderer(*ts.scene, BowlAtScale(1.0), 960, 640,
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
        OSGViewRenderer renderer(*ts.scene, BowlAtScale(1.0), 960, 640,
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
        OSGViewRenderer renderer(*ts.scene, BowlAtScale(1.0, cv::Vec2d(2.0, 1.0)),
                                 960, 640, RenderMode::Windowed));
}
