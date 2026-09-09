// Tests for avm_stitch mesh bake (per-vertex UV + blend weights).
// Pure math, no GL / OSG dependency.

#include <gtest/gtest.h>

#include "geometry/bowl_model.hpp"
#include "geometry/projection.hpp"
#include "geometry/mesh_bake.hpp"

#include <cmath>
#include <vector>

using namespace avm_stitch;

namespace {

CameraIntrinsics MakeIntrinsics() {
    return {1920, 1536, 441.45, 441.45, 952.14, 758.27, 0, 0, 0, 0};
}

// N identity-rotation cameras at azimuths (deg), radius 3m, height 1m.
// Looking along +Z (straight up): the bowl rim rising above z=1 is visible.
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

bool IsVertexVisibleToCamera(const cv::Vec3d& P, const CameraIntrinsics& intr,
                             const CameraExtrinsics& extr) {
    double u_px, v_px;
    ProjectWorldToFishEye(P, intr, extr, u_px, v_px);
    return IsForwardFacing(P, extr)
           && u_px >= 0.0 && u_px < intr.width
           && v_px >= 0.0 && v_px < intr.height;
}

BowlMesh MakeCoarseBowl(double r_outter) {
    BowlParams bowl = BowlParams::Default();
    bowl.step = 1.0;        // coarse grid for test speed
    bowl.r_outter = r_outter;
    return BowlMesh(bowl);
}

} // namespace

TEST(BakeMeshTest, SingleCamera_UVMatchesProjection) {
    CameraIntrinsics intr = MakeIntrinsics();
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)};
    auto mesh = MakeCoarseBowl(5.0);
    ASSERT_GT(mesh.vertex_count(), 0);

    auto baked = BakedMesh(mesh, {intr}, {extr});
    ASSERT_EQ(baked.vertex_count(), mesh.vertex_count());
    ASSERT_EQ(1, baked.num_cameras());

    int checked = 0;
    for (int i = 0; i < mesh.vertex_count(); ++i) {
        if (!IsVertexVisibleToCamera(mesh.vertex(i), intr, extr)) continue;
        ++checked;
        double u_px, v_px;
        ProjectWorldToFishEye(mesh.vertex(i), intr, extr, u_px, v_px);
        EXPECT_NEAR(baked.baked_vertex(i).uv[0][0], u_px / intr.width, 1e-6);
        // OpenCV/T_wc convention: camera y axis points DOWN, so v_px grows toward
        // the image bottom directly and is already a UV V coordinate. No flip.
        EXPECT_NEAR(baked.baked_vertex(i).uv[0][1], v_px / intr.height, 1e-6);
        EXPECT_NEAR(baked.baked_vertex(i).weight[0], 1.0, 1e-6);
    }
    EXPECT_GT(checked, 0);   // some vertices must actually be visible
}

TEST(BakeMeshTest, N4_VertexCountMatches) {
    auto mesh = MakeCoarseBowl(12.6);
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    auto baked = BakedMesh(mesh, intrs, extrs);
    ASSERT_EQ(baked.vertex_count(), mesh.vertex_count());
    ASSERT_EQ(4, baked.num_cameras());
}

TEST(BakeMeshTest, N4_NumCameras) {
    auto mesh = MakeCoarseBowl(12.6);
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    auto baked = BakedMesh(mesh, intrs, extrs);
    EXPECT_EQ(4, baked.num_cameras());
}

TEST(BakeMeshTest, N4_WeightsSumToOne_WhenVisible) {
    auto mesh = MakeCoarseBowl(12.6);
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    auto baked = BakedMesh(mesh, intrs, extrs);

    int visible_vertices = 0;
    for (int i = 0; i < baked.vertex_count(); ++i) {
        const BakedVertex& bv = baked.baked_vertex(i);
        double sum = 0.0;
        for (int j = 0; j < 4; ++j) sum += bv.weight[j];
        if (sum > 1e-9) {
            ++visible_vertices;
            EXPECT_NEAR(1.0, sum, 1e-6);
        }
    }
    EXPECT_GT(visible_vertices, 0);   // bowl rim (z > 1) must be visible
}

TEST(BakeMeshTest, N4_UVInNormalizedRange) {
    auto mesh = MakeCoarseBowl(12.6);
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    auto baked = BakedMesh(mesh, intrs, extrs);

    for (int i = 0; i < baked.vertex_count(); ++i) {
        const BakedVertex& bv = baked.baked_vertex(i);
        for (int j = 0; j < 4; ++j) {
            EXPECT_GE(bv.uv[j][0], 0.0f);
            EXPECT_LE(bv.uv[j][0], 1.0f);
            EXPECT_GE(bv.uv[j][1], 0.0f);
            EXPECT_LE(bv.uv[j][1], 1.0f);
        }
    }
}

TEST(BakeMeshTest, N4_InvisibleCameraHasZeroWeight) {
    auto mesh = MakeCoarseBowl(12.6);
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    auto baked = BakedMesh(mesh, intrs, extrs);

    // Every vertex that a camera reports as behind it must carry weight 0.
    // The flat centre (z=0) is below the z=1 camera ring: invisible to all.
    bool found_invisible_pair = false;
    bool found_flat_vertex = false;
    for (int i = 0; i < mesh.vertex_count(); ++i) {
        if (mesh.vertex(i)[2] == 0.0) found_flat_vertex = true;
        for (int j = 0; j < 4; ++j) {
            if (!IsVertexVisibleToCamera(mesh.vertex(i), intrs[j], extrs[j])) {
                found_invisible_pair = true;
                EXPECT_FLOAT_EQ(0.0f, baked.baked_vertex(i).weight[j]);
            }
        }
    }
    EXPECT_TRUE(found_invisible_pair);
    EXPECT_TRUE(found_flat_vertex);
}

TEST(BakeMeshTest, N4_AllVerticesHaveAtLeastOneVisible) {
    auto mesh = MakeCoarseBowl(12.6);
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    auto baked = BakedMesh(mesh, intrs, extrs);

    // The bowl rim rises above the camera ring -> at least some vertices get
    // blend weight; the flat centre is below it -> some stay invisible.
    int visible_vertices = 0;
    int invisible_vertices = 0;
    for (int i = 0; i < baked.vertex_count(); ++i) {
        const BakedVertex& bv = baked.baked_vertex(i);
        double sum = 0.0;
        for (int j = 0; j < 4; ++j) sum += bv.weight[j];
        if (sum > 1e-9) ++visible_vertices;
        else ++invisible_vertices;
    }
    EXPECT_GT(visible_vertices, 0);
    EXPECT_GT(invisible_vertices, 0);
}

TEST(BakeMeshTest, N8_AllSlotsFilledCorrectly) {
    auto mesh = MakeCoarseBowl(12.6);
    auto intrs = std::vector<CameraIntrinsics>(8, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 45, 90, 135, 180, 225, 270, 315});
    auto baked = BakedMesh(mesh, intrs, extrs);
    ASSERT_EQ(8, baked.num_cameras());

    bool slot7_used = false;
    for (int i = 0; i < baked.vertex_count(); ++i) {
        const BakedVertex& bv = baked.baked_vertex(i);
        double sum = 0.0;
        for (int j = 0; j < 8; ++j) sum += bv.weight[j];
        if (sum > 1e-9) EXPECT_NEAR(1.0, sum, 1e-6);
        EXPECT_GE(bv.weight[7], 0.0f);
        EXPECT_GE(bv.uv[7][0], 0.0f);
        EXPECT_GE(bv.uv[7][1], 0.0f);
        if (bv.weight[7] > 0.0f) slot7_used = true;
    }
    EXPECT_TRUE(slot7_used);   // the 8th camera must contribute somewhere
}

TEST(BakeMeshTest, N6_UnusedSlotsZero) {
    auto mesh = MakeCoarseBowl(12.6);
    auto intrs = std::vector<CameraIntrinsics>(6, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 60, 120, 180, 240, 300});
    auto baked = BakedMesh(mesh, intrs, extrs);
    ASSERT_EQ(6, baked.num_cameras());

    for (int i = 0; i < baked.vertex_count(); ++i) {
        const BakedVertex& bv = baked.baked_vertex(i);
        EXPECT_FLOAT_EQ(0.0f, bv.weight[6]);
        EXPECT_FLOAT_EQ(0.0f, bv.weight[7]);
        EXPECT_FLOAT_EQ(0.0f, bv.uv[6][0]);
        EXPECT_FLOAT_EQ(0.0f, bv.uv[6][1]);
        EXPECT_FLOAT_EQ(0.0f, bv.uv[7][0]);
        EXPECT_FLOAT_EQ(0.0f, bv.uv[7][1]);
    }
}

// A vertex whose fisheye projection lands inside the image RECTANGLE but
// OUTSIDE the fisheye effective circle (the black dead zone around the lens
// disc) must NOT be baked as visible — otherwise the stitcher samples black
// pixels and leaves a black seam between camera blocks.
TEST(BakeMeshTest, PointInRectButOutsideFisheyeCircle_Invisible) {
    // 640x480 camera, principal point at the image centre. Auto effective
    // circle radius = min(320, 240, 320, 240) = 240 px.
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    // Identity rotation, camera 5 m up +Z: points at world z=10 are 5 m in
    // front (P_cam z = 5).
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};

    // x_n = tan(0.3)  -> u ~ 407, v ~ 327: inside the rect AND inside the
    // circle (offset ~124 px < 240) -> must stay visible.
    cv::Vec3d inside(5.0 * std::tan(0.3), 5.0 * std::tan(0.3), 10.0);
    // x_n = tan(0.9)  -> u = 590, v = 240: inside the rect (590 < 640) but
    // 270 px off the principal point -> OUTSIDE the r=240 circle.
    cv::Vec3d outside(5.0 * std::tan(0.9), 0.0, 10.0);

    double u_in, v_in, u_out, v_out;
    ProjectWorldToFishEye(inside, intr, extr, u_in, v_in);
    ProjectWorldToFishEye(outside, intr, extr, u_out, v_out);

    // Sanity: both project inside the image rectangle — the old rect-only
    // visibility check would mark BOTH visible. Only the circle separates them.
    ASSERT_GE(u_in, 0.0);    ASSERT_LT(u_in, intr.width);
    ASSERT_GE(v_in, 0.0);    ASSERT_LT(v_in, intr.height);
    ASSERT_GE(u_out, 0.0);   ASSERT_LT(u_out, intr.width);
    ASSERT_GE(v_out, 0.0);   ASSERT_LT(v_out, intr.height);
    ASSERT_LT(std::hypot(u_in - intr.cx, v_in - intr.cy), 240.0);
    ASSERT_GT(std::hypot(u_out - intr.cx, v_out - intr.cy), 240.0);

    BowlMesh mesh({inside, outside},
                  {cv::Vec3d(0.0, 0.0, 1.0), cv::Vec3d(0.0, 0.0, 1.0)});

    auto baked = BakedMesh(mesh, {intr}, {extr});

    // In-circle point: visible with full weight and matching UV.
    EXPECT_NEAR(1.0, baked.baked_vertex(0).weight[0], 1e-6);
    EXPECT_NEAR(u_in / intr.width, baked.baked_vertex(0).uv[0][0], 1e-6);
    EXPECT_NEAR(v_in / intr.height, baked.baked_vertex(0).uv[0][1], 1e-6);

    // Out-of-circle point (still inside the rect): must stay invisible —
    // weight 0 and UV 0 (black dead zone, must not feed the blender).
    EXPECT_FLOAT_EQ(0.0f, baked.baked_vertex(1).weight[0]);
    EXPECT_FLOAT_EQ(0.0f, baked.baked_vertex(1).uv[0][0]);
    EXPECT_FLOAT_EQ(0.0f, baked.baked_vertex(1).uv[0][1]);
}

TEST(BakeMeshTest, NoCrashWithAllInvisible) {
    // Camera above the bowl looking straight up: every bowl vertex (z < 5)
    // is behind it -> nothing visible, weights stay all-zero, no crash.
    auto mesh = MakeCoarseBowl(5.0);
    CameraIntrinsics intr = MakeIntrinsics();
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};
    auto baked = BakedMesh(mesh, {intr}, {extr});
    ASSERT_EQ(baked.vertex_count(), mesh.vertex_count());

    for (int i = 0; i < baked.vertex_count(); ++i) {
        const BakedVertex& bv = baked.baked_vertex(i);
        EXPECT_FLOAT_EQ(0.0f, bv.weight[0]);
        EXPECT_FLOAT_EQ(0.0f, bv.uv[0][0]);
        EXPECT_FLOAT_EQ(0.0f, bv.uv[0][1]);
    }
}

// ---------------------------------------------------------------------------
// Fisheye-edge feather: near-circle samples must not pollute the seam blend.
// ---------------------------------------------------------------------------

// Feather preserves the single-camera invariant: any visible vertex normalizes
// to weight 1.0 even when its sample sits close to the fisheye circle edge.
TEST(BakeMeshTest, SingleCamera_NormalPointWeightOne) {
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};

    // World point that projects to the image centre (r_px = 0, feather = 1.0).
    BowlMesh mesh({cv::Vec3d(0.0, 0.0, 10.0)}, {cv::Vec3d(0.0, 0.0, 1.0)});
    auto baked = BakedMesh(mesh, {intr}, {extr});

    ASSERT_GT(baked.vertex_count(), 0);
    EXPECT_NEAR(1.0, baked.baked_vertex(0).weight[0], 1e-6);
}

// In the overlap of two cameras, the feather must attenuate the camera whose
// sample lands near its fisheye circle edge (where linear filtering would mix
// the black dead-zone into the seam) so the cleaner, centre-seeing camera
// dominates the blend.
TEST(BakeMeshTest, FeatherAttenuatesNearEdgeCameraInOverlap) {
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    CameraExtrinsics cam0{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};

    // World point seen by cam0 230 px off its principal point (10 px inside
    // the r=240 effective circle): theta = 230/300 rad -> r_u = tan(230/300),
    // P_cam z = 5. cam1 shifted to (r_u*5, 0, 5) sees it at its exact centre.
    const double r_u = std::tan(230.0 / 300.0);
    cv::Vec3d P(r_u * 5.0, 0.0, 10.0);
    CameraExtrinsics cam1{cv::Matx33d::eye(), cv::Vec3d(r_u * 5.0, 0.0, 5.0)};

    double u0, v0, u1, v1;
    ProjectWorldToFishEye(P, intr, cam0, u0, v0);
    ProjectWorldToFishEye(P, intr, cam1, u1, v1);
    ASSERT_NEAR(230.0, std::hypot(u0 - intr.cx, v0 - intr.cy), 1e-6);
    ASSERT_NEAR(0.0, std::hypot(u1 - intr.cx, v1 - intr.cy), 1e-6);

    BowlMesh mesh({P}, {cv::Vec3d(0.0, 0.0, 1.0)});
    auto baked = BakedMesh(mesh, {intr, intr}, {cam0, cam1});

    // Raw blend weights are symmetric (0.5 / 0.5); only the feather breaks the
    // tie: cam0's edge sample (feather ~0.625) must lose to cam1's centre one.
    EXPECT_LT(baked.baked_vertex(0).weight[0], baked.baked_vertex(0).weight[1]);
    EXPECT_LT(baked.baked_vertex(0).weight[0], 0.5f);
    EXPECT_GT(baked.baked_vertex(0).weight[1], 0.5f);
    // Normalization invariant is preserved.
    EXPECT_NEAR(1.0, baked.baked_vertex(0).weight[0] + baked.baked_vertex(0).weight[1], 1e-6);
}

// ---------------------------------------------------------------------------
// Fine-grained index access + semantic queries (no container exposure).
// ---------------------------------------------------------------------------

TEST(BakeMeshTest, IndexAccess_CountsMatchBowl) {
    auto mesh = MakeCoarseBowl(12.6);
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    auto baked = BakedMesh(mesh, intrs, extrs);
    EXPECT_EQ(mesh.vertex_count(), baked.vertex_count());
}

TEST(BakeMeshTest, IndexAccess_BakedVertexWeightSemantics) {
    auto mesh = MakeCoarseBowl(12.6);
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    auto baked = BakedMesh(mesh, intrs, extrs);

    // Same weight-sum-to-one expectation as the loop form, via index access.
    int visible_vertices = 0;
    for (int i = 0; i < baked.vertex_count(); ++i) {
        const BakedVertex& bv = baked.baked_vertex(i);
        double sum = 0.0;
        for (int j = 0; j < 4; ++j) sum += bv.weight[j];
        if (sum > 1e-9) {
            ++visible_vertices;
            EXPECT_NEAR(1.0, sum, 1e-6);
        }
    }
    EXPECT_GT(visible_vertices, 0);
}

TEST(BakeMeshTest, HasVisibleVertices_TrueWhenRimVisible) {
    // Bowl rim (z > 1) rises above the z=1 camera ring -> visible.
    auto mesh = MakeCoarseBowl(12.6);
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    auto baked = BakedMesh(mesh, intrs, extrs);
    EXPECT_TRUE(baked.HasVisibleVertices());
}

TEST(BakeMeshTest, HasVisibleVertices_FalseWhenAllInvisible) {
    // Camera above the bowl looking straight up: every bowl vertex (z < 5)
    // is behind it -> no visible vertex anywhere.
    auto mesh = MakeCoarseBowl(5.0);
    CameraIntrinsics intr = MakeIntrinsics();
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};
    auto baked = BakedMesh(mesh, {intr}, {extr});
    EXPECT_FALSE(baked.HasVisibleVertices());
}
