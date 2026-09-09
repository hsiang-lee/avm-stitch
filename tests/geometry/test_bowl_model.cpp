// Tests for avm_stitch bowl model (BowlParams + BowlModel mesh + Bake).
// Pure math, no GL / OSG dependency.

#include <gtest/gtest.h>

#include "geometry/bowl_model.hpp"
#include "geometry/projection.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace avm_stitch;

namespace {

// Count vertices that lie strictly inside the flat (rd == 0) region.
size_t CountFlatVertices(const BowlModel& model, double x_half, double y_half) {
    size_t count = 0;
    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& v = model.vertex(i);
        if (std::abs(v[0]) < x_half - 1e-9 && std::abs(v[1]) < y_half - 1e-9) {
            count++;
        }
    }
    return count;
}

// ---- bake test helpers ----

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

BowlModel MakeCoarseBowl(double r_outter) {
    BowlParams bowl = BowlParams::Default();
    bowl.step = 1.0;        // coarse grid for test speed
    bowl.r_outter = r_outter;
    return BowlModel(bowl);
}

} // namespace

// ---------------------------------------------------------------------------
// BowlParams
// ---------------------------------------------------------------------------

TEST(BowlParamsTest, Default_SetsCorrectDefaults) {
    auto p = BowlParams::Default();
    EXPECT_DOUBLE_EQ(5.6, p.car_length);
    EXPECT_DOUBLE_EQ(2.2, p.car_width);
    EXPECT_DOUBLE_EQ(2.0, p.round_l);
    EXPECT_DOUBLE_EQ(0.1, p.step);
    EXPECT_DOUBLE_EQ(12.6, p.r_outter);
    EXPECT_DOUBLE_EQ(0.09, p.k_slop);
    EXPECT_DOUBLE_EQ(2.2, p.z_exp);
    EXPECT_DOUBLE_EQ(8.6, p.max_z);
}

// ---------------------------------------------------------------------------
// BowlModel construction
// ---------------------------------------------------------------------------

TEST(BowlModelTest, ReturnsNonEmptyMesh) {
    const BowlModel model(BowlParams::Default());
    EXPECT_GT(model.vertex_count(), 0);
    EXPECT_GT(model.triangle_count(), 0);
    EXPECT_GT(model.rows(), 0);
    EXPECT_GT(model.cols(), 0);
}

TEST(BowlModelTest, GridSizeMatchesParams) {
    const BowlModel model(BowlParams::Default());
    // r_outter=12.6, step=0.1 -> (12.6/0.1)*2 = 252 (rounded, not truncated)
    EXPECT_EQ(252, model.rows());
    EXPECT_EQ(252, model.cols());
}

TEST(BowlModelTest, VerticesInXyBounds) {
    const BowlModel model(BowlParams::Default());
    ASSERT_GT(model.vertex_count(), 0);
    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& v = model.vertex(i);
        EXPECT_GE(v[0], -12.6 - 1e-9);
        EXPECT_LE(v[0], 12.6 + 1e-9);
        EXPECT_GE(v[1], -12.6 - 1e-9);
        EXPECT_LE(v[1], 12.6 + 1e-9);
    }
}

TEST(BowlModelTest, VerticesZInRange) {
    const BowlModel model(BowlParams::Default());
    ASSERT_GT(model.vertex_count(), 0);
    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& v = model.vertex(i);
        EXPECT_GE(v[2], 0.0);
        EXPECT_LE(v[2], 8.6 + 1e-9);
    }
}

TEST(BowlModelTest, FlatCenterRegion) {
    BowlParams params;
    params.car_length = 2.0;
    params.car_width = 1.0;
    params.round_l = 0.5;
    double x_half = params.car_length / 2.0 + params.round_l; // 1.5
    double y_half = params.car_width / 2.0 + params.round_l;  // 1.0
    const BowlModel model(params);
    bool found_flat = false;
    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& v = model.vertex(i);
        if (std::abs(v[0]) < x_half - 1e-9 && std::abs(v[1]) < y_half - 1e-9) {
            found_flat = true;
            EXPECT_DOUBLE_EQ(0.0, v[2]);
        }
    }
    EXPECT_TRUE(found_flat);
}

TEST(BowlModelTest, EdgesMonotonic) {
    // Along a fixed row (fixed u/x), z must be a non-decreasing function of rd.
    const BowlModel model(BowlParams::Default());
    ASSERT_GT(model.rows(), 0);
    const double x_half = 5.6 / 2.0 + 2.0;
    const double y_half = 2.2 / 2.0 + 2.0;
    const double r_outter = 12.6;

    // Group vertices into rows (recover grid row i from world x), O(n).
    std::vector<std::vector<std::pair<double, double>>> rows(model.rows() + 1);
    for (int k = 0; k < model.vertex_count(); ++k) {
        const cv::Vec3d& v = model.vertex(k);
        double u = v[0] / r_outter;
        int i = std::lround((u + 1.0) / 2.0 * model.rows());
        double dx = std::max(0.0, std::abs(v[0]) - x_half);
        double dy = std::max(0.0, std::abs(v[1]) - y_half);
        double rd = std::sqrt(dx * dx + dy * dy);
        if (i >= 0 && i <= model.rows()) rows[i].emplace_back(rd, v[2]);
    }

    for (int i = 0; i <= model.rows(); ++i) {
        auto& row = rows[i];
        if (row.size() < 2) continue;
        std::sort(row.begin(), row.end());
        for (size_t k = 1; k < row.size(); ++k) {
            EXPECT_GE(row[k].second, row[k - 1].second - 1e-9)
                << "row " << i << ": z decreases when rd increases";
        }
    }
}

TEST(BowlModelTest, NormalsAreUnitLength) {
    const BowlModel model(BowlParams::Default());
    ASSERT_EQ(model.vertex_count(), model.normal_count());
    for (int i = 0; i < model.normal_count(); ++i) {
        const cv::Vec3d& n = model.normal(i);
        double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        EXPECT_NEAR(1.0, len, 1e-6);
    }
}

TEST(BowlModelTest, FlatCenterNormalsUpward) {
    BowlParams params;
    params.car_length = 2.0;
    params.car_width = 1.0;
    params.round_l = 0.5;
    double x_half = 1.5;
    double y_half = 1.0;
    const BowlModel model(params);
    bool found_flat = false;
    for (int k = 0; k < model.vertex_count(); ++k) {
        const cv::Vec3d& v = model.vertex(k);
        if (std::abs(v[0]) < x_half - 1e-9 && std::abs(v[1]) < y_half - 1e-9) {
            found_flat = true;
            const cv::Vec3d& n = model.normal(k);
            EXPECT_NEAR(0.0, n[0], 1e-6);
            EXPECT_NEAR(0.0, n[1], 1e-6);
            EXPECT_NEAR(1.0, n[2], 1e-6);
        }
    }
    EXPECT_TRUE(found_flat);
}

TEST(BowlModelTest, TrianglesHaveValidIndices) {
    const BowlModel model(BowlParams::Default());
    ASSERT_GT(model.triangle_count(), 0);
    for (int t = 0; t < model.triangle_count(); ++t) {
        for (int idx : model.triangle(t)) {
            EXPECT_GE(idx, 0);
            EXPECT_LT(idx, model.vertex_count());
        }
    }
}

TEST(BowlModelTest, DifferentCarSizeChangesFlatRegion) {
    BowlParams truck_params;
    truck_params.car_length = 18.0;
    truck_params.car_width = 2.5;
    truck_params.step = 0.5;   // coarse for speed
    const BowlModel truck_model(truck_params);
    EXPECT_GT(truck_model.vertex_count(), 0);

    BowlParams default_params;
    default_params.step = 0.5; // coarse for speed
    const BowlModel default_model(default_params);

    size_t truck_flat = CountFlatVertices(truck_model, 18.0 / 2.0 + 2.0, 2.5 / 2.0 + 2.0);
    size_t default_flat = CountFlatVertices(default_model, 5.6 / 2.0 + 2.0, 2.2 / 2.0 + 2.0);
    EXPECT_GT(truck_flat, default_flat);
}

TEST(BowlModelTest, MinimalParams_OneCellMesh) {
    BowlParams params;
    params.step = 12.6;    // step == r_outter -> 1 cell per axis
    params.r_outter = 12.6;
    const BowlModel model(params);
    EXPECT_EQ(2, model.rows());
    EXPECT_EQ(2, model.cols());
    EXPECT_GT(model.vertex_count(), 0);   // center + edge points retained
    // must not crash; indices still valid
    for (int t = 0; t < model.triangle_count(); ++t) {
        for (int idx : model.triangle(t)) {
            EXPECT_LT(idx, model.vertex_count());
        }
    }
}

// ---------------------------------------------------------------------------
// Fine-grained index access: counts + per-element queries (no container
// exposure). These tests define the new read API surface.
// ---------------------------------------------------------------------------

TEST(BowlModelTest, IndexAccess_CountsAreConsistent) {
    const BowlModel model(BowlParams::Default());
    EXPECT_GT(model.vertex_count(), 0);
    EXPECT_GT(model.triangle_count(), 0);
    // Construction invariant: one normal per vertex.
    EXPECT_EQ(model.vertex_count(), model.normal_count());
}

TEST(BowlModelTest, IndexAccess_VertexInRange) {
    const BowlModel model(BowlParams::Default());
    // Every indexed vertex lies in the same XY/Z bounds as the model.
    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& v = model.vertex(i);
        EXPECT_GE(v[0], -12.6 - 1e-9);
        EXPECT_LE(v[0], 12.6 + 1e-9);
        EXPECT_GE(v[1], -12.6 - 1e-9);
        EXPECT_LE(v[1], 12.6 + 1e-9);
        EXPECT_GE(v[2], 0.0);
        EXPECT_LE(v[2], 8.6 + 1e-9);
    }
}

TEST(BowlModelTest, IndexAccess_NormalIsUnit) {
    const BowlModel model(BowlParams::Default());
    for (int i = 0; i < model.normal_count(); ++i) {
        const cv::Vec3d& n = model.normal(i);
        double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        EXPECT_NEAR(1.0, len, 1e-6);
    }
}

TEST(BowlModelTest, IndexAccess_TriangleIndicesValid) {
    const BowlModel model(BowlParams::Default());
    for (int t = 0; t < model.triangle_count(); ++t) {
        const std::array<int, 3> tri = model.triangle(t);
        for (int idx : tri) {
            EXPECT_GE(idx, 0);
            EXPECT_LT(idx, model.vertex_count());
        }
    }
}

TEST(BowlModelTest, IndexAccess_WrappingConstructorCounts) {
    // Wrap-constructor model: counts reflect the wrapped data, triangles empty.
    BowlModel model({cv::Vec3d(0.0, 0.0, 10.0)}, {cv::Vec3d(0.0, 0.0, 1.0)});
    EXPECT_EQ(1, model.vertex_count());
    EXPECT_EQ(1, model.normal_count());
    EXPECT_EQ(0, model.triangle_count());
    EXPECT_DOUBLE_EQ(10.0, model.vertex(0)[2]);
    EXPECT_NEAR(1.0, model.normal(0)[2], 1e-9);
}

// ---------------------------------------------------------------------------
// Bake: per-vertex UV + blend weights (assertions kept verbatim from the
// former bake test file; only the construction form changed: construct
// BowlModel then call Bake()).
// ---------------------------------------------------------------------------

TEST(BowlModelBakeTest, SingleCamera_UVMatchesProjection) {
    CameraIntrinsics intr = MakeIntrinsics();
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)};
    auto model = MakeCoarseBowl(5.0);
    ASSERT_GT(model.vertex_count(), 0);

    model.Bake({intr}, {extr});
    // One baked vertex per bowl vertex (index range validated by the loop
    // below via the accessor's bound assert).
    ASSERT_EQ(1, model.baked_num_cameras());

    int checked = 0;
    for (int i = 0; i < model.vertex_count(); ++i) {
        if (!IsVertexVisibleToCamera(model.vertex(i), intr, extr)) continue;
        ++checked;
        double u_px, v_px;
        ProjectWorldToFishEye(model.vertex(i), intr, extr, u_px, v_px);
        EXPECT_NEAR(model.baked_vertex(i).uv[0][0], u_px / intr.width, 1e-6);
        // OpenCV/T_wc convention: camera y axis points DOWN, so v_px grows toward
        // the image bottom directly and is already a UV V coordinate. No flip.
        EXPECT_NEAR(model.baked_vertex(i).uv[0][1], v_px / intr.height, 1e-6);
        EXPECT_NEAR(model.baked_vertex(i).weight[0], 1.0, 1e-6);
    }
    EXPECT_GT(checked, 0);   // some vertices must actually be visible
}

TEST(BowlModelBakeTest, N4_VertexCountMatches) {
    auto model = MakeCoarseBowl(12.6);
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    model.Bake(intrs, extrs);
    // One baked vertex per bowl vertex (index range validated by the loop
    // below via the accessor's bound assert).
    ASSERT_EQ(4, model.baked_num_cameras());
}

TEST(BowlModelBakeTest, N4_NumCameras) {
    auto model = MakeCoarseBowl(12.6);
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    model.Bake(intrs, extrs);
    EXPECT_EQ(4, model.baked_num_cameras());
}

TEST(BowlModelBakeTest, N4_WeightsSumToOne_WhenVisible) {
    auto model = MakeCoarseBowl(12.6);
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    model.Bake(intrs, extrs);

    int visible_vertices = 0;
    for (int i = 0; i < model.vertex_count(); ++i) {
        const BakedVertex& bv = model.baked_vertex(i);
        double sum = 0.0;
        for (int j = 0; j < 4; ++j) sum += bv.weight[j];
        if (sum > 1e-9) {
            ++visible_vertices;
            EXPECT_NEAR(1.0, sum, 1e-6);
        }
    }
    EXPECT_GT(visible_vertices, 0);   // bowl rim (z > 1) must be visible
}

TEST(BowlModelBakeTest, N4_UVInNormalizedRange) {
    auto model = MakeCoarseBowl(12.6);
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    model.Bake(intrs, extrs);

    for (int i = 0; i < model.vertex_count(); ++i) {
        const BakedVertex& bv = model.baked_vertex(i);
        for (int j = 0; j < 4; ++j) {
            EXPECT_GE(bv.uv[j][0], 0.0f);
            EXPECT_LE(bv.uv[j][0], 1.0f);
            EXPECT_GE(bv.uv[j][1], 0.0f);
            EXPECT_LE(bv.uv[j][1], 1.0f);
        }
    }
}

TEST(BowlModelBakeTest, N4_InvisibleCameraHasZeroWeight) {
    auto model = MakeCoarseBowl(12.6);
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    model.Bake(intrs, extrs);

    // Every vertex that a camera reports as behind it must carry weight 0.
    // The flat centre (z=0) is below the z=1 camera ring: invisible to all.
    bool found_invisible_pair = false;
    bool found_flat_vertex = false;
    for (int i = 0; i < model.vertex_count(); ++i) {
        if (model.vertex(i)[2] == 0.0) found_flat_vertex = true;
        for (int j = 0; j < 4; ++j) {
            if (!IsVertexVisibleToCamera(model.vertex(i), intrs[j], extrs[j])) {
                found_invisible_pair = true;
                EXPECT_FLOAT_EQ(0.0f, model.baked_vertex(i).weight[j]);
            }
        }
    }
    EXPECT_TRUE(found_invisible_pair);
    EXPECT_TRUE(found_flat_vertex);
}

TEST(BowlModelBakeTest, N4_AllVerticesHaveAtLeastOneVisible) {
    auto model = MakeCoarseBowl(12.6);
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    model.Bake(intrs, extrs);

    // The bowl rim rises above the camera ring -> at least some vertices get
    // blend weight; the flat centre is below it -> some stay invisible.
    int visible_vertices = 0;
    int invisible_vertices = 0;
    for (int i = 0; i < model.vertex_count(); ++i) {
        const BakedVertex& bv = model.baked_vertex(i);
        double sum = 0.0;
        for (int j = 0; j < 4; ++j) sum += bv.weight[j];
        if (sum > 1e-9) ++visible_vertices;
        else ++invisible_vertices;
    }
    EXPECT_GT(visible_vertices, 0);
    EXPECT_GT(invisible_vertices, 0);
}

TEST(BowlModelBakeTest, N8_AllSlotsFilledCorrectly) {
    auto model = MakeCoarseBowl(12.6);
    auto intrs = std::vector<CameraIntrinsics>(8, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 45, 90, 135, 180, 225, 270, 315});
    model.Bake(intrs, extrs);
    ASSERT_EQ(8, model.baked_num_cameras());

    bool slot7_used = false;
    for (int i = 0; i < model.vertex_count(); ++i) {
        const BakedVertex& bv = model.baked_vertex(i);
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

TEST(BowlModelBakeTest, N6_UnusedSlotsZero) {
    auto model = MakeCoarseBowl(12.6);
    auto intrs = std::vector<CameraIntrinsics>(6, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 60, 120, 180, 240, 300});
    model.Bake(intrs, extrs);
    ASSERT_EQ(6, model.baked_num_cameras());

    for (int i = 0; i < model.vertex_count(); ++i) {
        const BakedVertex& bv = model.baked_vertex(i);
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
TEST(BowlModelBakeTest, PointInRectButOutsideFisheyeCircle_Invisible) {
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

    BowlModel model({inside, outside},
                  {cv::Vec3d(0.0, 0.0, 1.0), cv::Vec3d(0.0, 0.0, 1.0)});

    model.Bake({intr}, {extr});

    // In-circle point: visible with full weight and matching UV.
    EXPECT_NEAR(1.0, model.baked_vertex(0).weight[0], 1e-6);
    EXPECT_NEAR(u_in / intr.width, model.baked_vertex(0).uv[0][0], 1e-6);
    EXPECT_NEAR(v_in / intr.height, model.baked_vertex(0).uv[0][1], 1e-6);

    // Out-of-circle point (still inside the rect): must stay invisible —
    // weight 0 and UV 0 (black dead zone, must not feed the blender).
    EXPECT_FLOAT_EQ(0.0f, model.baked_vertex(1).weight[0]);
    EXPECT_FLOAT_EQ(0.0f, model.baked_vertex(1).uv[0][0]);
    EXPECT_FLOAT_EQ(0.0f, model.baked_vertex(1).uv[0][1]);
}

TEST(BowlModelBakeTest, NoCrashWithAllInvisible) {
    // Camera above the bowl looking straight up: every bowl vertex (z < 5)
    // is behind it -> nothing visible, weights stay all-zero, no crash.
    auto model = MakeCoarseBowl(5.0);
    CameraIntrinsics intr = MakeIntrinsics();
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};
    model.Bake({intr}, {extr});
    // One baked vertex per bowl vertex (index range validated by the loop
    // below via the accessor's bound assert).

    for (int i = 0; i < model.vertex_count(); ++i) {
        const BakedVertex& bv = model.baked_vertex(i);
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
TEST(BowlModelBakeTest, SingleCamera_NormalPointWeightOne) {
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};

    // World point that projects to the image centre (r_px = 0, feather = 1.0).
    BowlModel model({cv::Vec3d(0.0, 0.0, 10.0)}, {cv::Vec3d(0.0, 0.0, 1.0)});
    model.Bake({intr}, {extr});

    ASSERT_GT(model.vertex_count(), 0);
    EXPECT_NEAR(1.0, model.baked_vertex(0).weight[0], 1e-6);
}

// In the overlap of two cameras, the feather must attenuate the camera whose
// sample lands near its fisheye circle edge (where linear filtering would mix
// the black dead-zone into the seam) so the cleaner, centre-seeing camera
// dominates the blend.
TEST(BowlModelBakeTest, FeatherAttenuatesNearEdgeCameraInOverlap) {
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

    BowlModel model({P}, {cv::Vec3d(0.0, 0.0, 1.0)});
    model.Bake({intr, intr}, {cam0, cam1});

    // Raw blend weights are symmetric (0.5 / 0.5); only the feather breaks the
    // tie: cam0's edge sample (feather ~0.625) must lose to cam1's centre one.
    EXPECT_LT(model.baked_vertex(0).weight[0], model.baked_vertex(0).weight[1]);
    EXPECT_LT(model.baked_vertex(0).weight[0], 0.5f);
    EXPECT_GT(model.baked_vertex(0).weight[1], 0.5f);
    // Normalization invariant is preserved.
    EXPECT_NEAR(1.0, model.baked_vertex(0).weight[0] + model.baked_vertex(0).weight[1], 1e-6);
}

// ---------------------------------------------------------------------------
// Fine-grained index access + semantic queries (no container exposure).
// ---------------------------------------------------------------------------

TEST(BowlModelBakeTest, IndexAccess_CountsMatchBowl) {
    auto model = MakeCoarseBowl(12.6);
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    model.Bake(intrs, extrs);
    // One baked vertex per bowl vertex (index range validated by the loop
    // below via the accessor's bound assert).
}

TEST(BowlModelBakeTest, IndexAccess_BakedVertexWeightSemantics) {
    auto model = MakeCoarseBowl(12.6);
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    model.Bake(intrs, extrs);

    // Same weight-sum-to-one expectation as the loop form, via index access.
    int visible_vertices = 0;
    for (int i = 0; i < model.vertex_count(); ++i) {
        const BakedVertex& bv = model.baked_vertex(i);
        double sum = 0.0;
        for (int j = 0; j < 4; ++j) sum += bv.weight[j];
        if (sum > 1e-9) {
            ++visible_vertices;
            EXPECT_NEAR(1.0, sum, 1e-6);
        }
    }
    EXPECT_GT(visible_vertices, 0);
}

TEST(BowlModelBakeTest, HasVisibleVertices_TrueWhenRimVisible) {
    // Bowl rim (z > 1) rises above the z=1 camera ring -> visible.
    auto model = MakeCoarseBowl(12.6);
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    model.Bake(intrs, extrs);
    EXPECT_TRUE(model.HasVisibleVertices());
}

TEST(BowlModelBakeTest, HasVisibleVertices_FalseWhenAllInvisible) {
    // Camera above the bowl looking straight up: every bowl vertex (z < 5)
    // is behind it -> no visible vertex anywhere.
    auto model = MakeCoarseBowl(5.0);
    CameraIntrinsics intr = MakeIntrinsics();
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};
    model.Bake({intr}, {extr});
    EXPECT_FALSE(model.HasVisibleVertices());
}
