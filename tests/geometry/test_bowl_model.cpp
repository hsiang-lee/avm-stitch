// Tests for avm_stitch bowl model (BowlParams + BowlModel mesh + bake).
// Pure math, no GL / OSG dependency.

#include <gtest/gtest.h>

#include "geometry/bowl_model.hpp"
#include "geometry/projection.hpp"

#include "test_support/stitch_test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace avm_stitch;

// Identical helper bodies (MakeSideCameras / MakeCoarseBowl) live in
// test_support/stitch_test_helpers.hpp; MakeIntrinsics stays local because
// this suite's historical value is the 1920x1536 distortion camera, not the
// renderer suite's 640x480 one.
using avm_stitch::test::MakeCarBowl;
using avm_stitch::test::MakeCoarseBowl;
using avm_stitch::test::MakeSideCameras;

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

bool IsVertexVisibleToCamera(const cv::Vec3d& P, const CameraIntrinsics& intr,
                             const CameraExtrinsics& extr) {
    // D3: visibility matches BakeVertices criteria — rect + circle check only.
    // IsForwardFacing is NOT used (hand-written projection handles all z).
    double u_px, v_px;
    ProjectWorldToFishEye(P, intr, extr, u_px, v_px);
    return u_px >= 0.0 && u_px < intr.width
           && v_px >= 0.0 && v_px < intr.height
           && IsInsideFisheyeCircle(intr, u_px, v_px);
}

// 4 side cameras: the default camera set for mesh-construction tests (the
// bake that runs at construction must not affect any mesh assertion).
std::vector<CameraIntrinsics> DefaultIntrinsics() {
    return std::vector<CameraIntrinsics>(4, MakeIntrinsics());
}

std::vector<CameraExtrinsics> DefaultBakeCameras() {
    return MakeSideCameras({0, 180, 90, -90});
}

// Car anchor with car_center pinned to (0,0): the pre-D1 baseline layout.
// MakeCarBowl() (shared, stitch_test_helpers.hpp) is the derived
// reference-car bowl; tests asserting exact historical numbers (12.6 grid
// extent etc.) re-pin the exact fields.
BowlParams CarBowlCentered() {
    BowlParams p = MakeCarBowl();
    p.car_center = cv::Vec2d(0.0, 0.0);
    return p;
}

} // namespace

// ---------------------------------------------------------------------------
// BowlParams
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// DeriveBowlParams — all bowl parameters derive from the vehicle's real
// dimensions (length/width/axis_x). One formula for cars and ships alike;
// there are no hardcoded per-vehicle presets (BowlParams::Default is gone).
// Invariants below pin the NEW equal-scaling family: s = L/5.6 is the SINGLE
// scale factor, every length ∝ s and k_slop ∝ s^-1.2 → the surface shape is
// exactly self-similar (isotropic rim heights, constant 63% opening, constant
// 126x126 grid at any vehicle size).
// ---------------------------------------------------------------------------

// Car-scale regression: L=5.6/W=2.2 must reproduce the historically healthy
// car bowl (r_outter=12.6, round_l=2.0, step=0.2, k_slop=0.09, max_z=8.6,
// z_exp=2.2) — otherwise 4-camera stitching degrades. z_exp is a fixed shape
// constant: the original steep-wall bowl silhouette.
TEST(DeriveBowlParamsTest, CarScale_MatchesCalibratedValues) {
    const BowlParams p = DeriveBowlParams(5.6, 2.2, 2.9);
    EXPECT_DOUBLE_EQ(5.6, p.car_length);
    EXPECT_DOUBLE_EQ(2.2, p.car_width);
    EXPECT_NEAR(12.6, p.r_outter, 12.6 * 0.20);
    EXPECT_NEAR(2.0, p.round_l, 2.0 * 0.20);
    EXPECT_NEAR(0.2, p.step, 0.2 * 0.20);
    EXPECT_NEAR(0.09, p.k_slop, 0.09 * 0.20);
    EXPECT_DOUBLE_EQ(2.2, p.z_exp);
    EXPECT_NEAR(8.6, p.max_z, 8.6 * 0.20);
    // car_center = (axis_x/2, 0) — rear-axle origin convention.
    EXPECT_DOUBLE_EQ(1.45, p.car_center[0]);
    EXPECT_DOUBLE_EQ(0.0, p.car_center[1]);
    EXPECT_EQ(BowlType::BlockBoat, p.type);
}

// Anchor EXACT regression: at s=1 (L=5.6) the six shape fields must be
// bit-exact equal to the old calibrated Default. The steep-wall silhouette is
// pinned too: rd* = (max_z/k_slop)^(1/2.2) ≈ 7.95 = 0.63*r_outter — the wall
// climbs to max_z BEFORE the grid edge and clips ("steep wall + opening",
// never a shallow full-diameter plate).
TEST(DeriveBowlParamsTest, CarAnchor_ExactRegressionToOldDefault) {
    const BowlParams p = DeriveBowlParams(5.6, 2.2, 2.9);
    EXPECT_DOUBLE_EQ(12.6, p.r_outter);
    EXPECT_DOUBLE_EQ(2.0, p.round_l);
    EXPECT_DOUBLE_EQ(0.2, p.step);
    EXPECT_DOUBLE_EQ(8.6, p.max_z);
    EXPECT_DOUBLE_EQ(0.09, p.k_slop);
    EXPECT_DOUBLE_EQ(2.2, p.z_exp);
    // Derived quantities: same surface equation as the mesh generator; pow()
    // introduces ~1e-5 ulp error, so assert with a tight absolute band.
    const double rd_star = std::pow(p.max_z / p.k_slop, 1.0 / p.z_exp);
    EXPECT_NEAR(7.9454, rd_star, 1e-4);
    EXPECT_NEAR(0.6306, rd_star / p.r_outter, 1e-4);
    EXPECT_LT(rd_star, p.r_outter);   // clipped steep wall
    // car_center = (axis_x/2, 0) — rear-axle origin convention.
    EXPECT_DOUBLE_EQ(1.45, p.car_center[0]);
    EXPECT_DOUBLE_EQ(0.0, p.car_center[1]);
    EXPECT_EQ(BowlType::BlockBoat, p.type);
}

// Linear similarity: the shape must be scale-free, so doubling the vehicle
// length EXACTLY doubles every length field, leaves z_exp untouched, and
// keeps the power-law curvature coefficients in lockstep (k_slop × 2^-1.2).
// This is the core assertion behind "等比例缩放" — the old sub-linear
// s^0.72/s^0.55 family fails it (r_outter grew 13.7x for a 37.8x ship).
TEST(DeriveBowlParamsTest, LinearSimilarity_DoublingLengthDoublesAllLengths) {
    const BowlParams p1 = DeriveBowlParams(5.6, 2.2, 2.9);    // s = 1
    const BowlParams p2 = DeriveBowlParams(11.2, 2.2, 2.9);   // s = 2
    ASSERT_GT(p1.k_slop, 0.0);
    ASSERT_GT(p1.max_z, 0.0);
    ASSERT_GT(p1.step, 0.0);
    ASSERT_GT(p1.round_l, 0.0);
    ASSERT_GT(p1.r_outter, 0.0);
    EXPECT_NEAR(2.0, p2.r_outter / p1.r_outter, 1e-9);
    EXPECT_NEAR(2.0, p2.round_l / p1.round_l, 1e-9);
    EXPECT_NEAR(2.0, p2.step / p1.step, 1e-9);
    EXPECT_NEAR(2.0, p2.max_z / p1.max_z, 1e-9);
    EXPECT_DOUBLE_EQ(p1.z_exp, p2.z_exp);            // shape exponent unchanged
    EXPECT_DOUBLE_EQ(2.2, p1.z_exp);
    EXPECT_NEAR(std::pow(2.0, -1.2), p2.k_slop / p1.k_slop, 1e-9);
}

// Self-similarity: the opening ratio rd*/r_outter must be the SAME constant
// (~0.63, the calibrated car anchor) at every scale — the wall always clips
// at 63% of the grid radius, so the "steep wall + opening" silhouette is
// preserved and never degenerates into a shallow plate at large scale.
// (Old family grew the opening to ~90% at the ship: each scale was clipped
// differently — the misleading comment this round corrects.)
TEST(DeriveBowlParamsTest, SelfSimilarity_OpeningRatioConstantAcrossScales) {
    const std::vector<double> scales = {5.6, 11.2, 44.8, 211.583};
    double ref_ratio = 0.0;
    for (double L : scales) {
        const BowlParams p = DeriveBowlParams(L, 2.2, L / 2.0);
        const double rd_star = std::pow(p.max_z / p.k_slop, 1.0 / p.z_exp);
        const double opening = rd_star / p.r_outter;
        if (ref_ratio == 0.0) {
            ref_ratio = opening;
            EXPECT_NEAR(0.63, ref_ratio, 0.63 * 0.01) << "anchor opening";
        } else {
            EXPECT_NEAR(ref_ratio, opening, ref_ratio * 1e-9) << "L=" << L;
        }
        EXPECT_LT(rd_star, p.r_outter) << "L=" << L;   // clipped steep wall
    }
}

// Isotropic rim heights (NEW — guards against the ship degenerating back
// into a trough): at the grid edge, the wall height in the longitudinal
// (bow/stern) direction and in the lateral (side) direction must agree
// within 15% using the SAME surface the mesh generator bakes
// (z = k_slop*rd^z_exp, clipped at max_z exactly like the vertex drop).
// The old sub-linear ship bowl had longitudinal rim ~3 m vs lateral ~35 m
// (11.5:1), i.e. "两侧起墙、船头尾全平".
TEST(DeriveBowlParamsTest, IsotropicRimHeight_LongitudinalVsLateral) {
    struct Car { double length, width, axis_x; };
    const Car kCars[] = {
        {5.6, 2.2, 2.9},        // car anchor
        {5.27, 2.17, 2.9},      // data_4
        {211.583, 53.453, 116.371},   // data_8 ship
    };
    for (const auto& c : kCars) {
        const BowlParams p = DeriveBowlParams(c.length, c.width, c.axis_x);
        auto rim_z = [&p](double rd) {
            const double z = p.k_slop * std::pow(rd, p.z_exp);
            return std::min(z, p.max_z);   // mesh clip: z > max_z dropped
        };
        const double rd_long = p.r_outter - (p.car_length / 2.0 + p.round_l);
        const double rd_lat = p.r_outter - (p.car_width / 2.0 + p.round_l);
        ASSERT_GT(rd_long, 0.0) << "L=" << c.length;
        ASSERT_GT(rd_lat, 0.0) << "L=" << c.length;
        const double z_long = rim_z(rd_long);
        const double z_lat = rim_z(rd_lat);
        const double ratio = std::max(z_long, z_lat) / std::min(z_long, z_lat);
        EXPECT_GE(ratio, 1.0) << "L=" << c.length;
        EXPECT_LE(ratio, 1.15) << "L=" << c.length
            << " z_long=" << z_long << " z_lat=" << z_lat;
    }
}

// Ship scale (L=211.6): the bowl must reach the farthest camera — cam_7 sits
// 149.8 m from car_center — so r_outter >= 150 is a hard requirement, and the
// opening radius rd* must clear the same camera with margin (steep-wall clip
// cannot shrink below the mesh reach, or the rim flattens before the grid
// edge).
TEST(DeriveBowlParamsTest, ShipScale_CoversCameras) {
    const BowlParams p = DeriveBowlParams(211.583, 53.453, 116.371);
    EXPECT_GE(p.r_outter, 150.0);
    const double rd_star = std::pow(p.max_z / p.k_slop, 1.0 / p.z_exp);
    EXPECT_GE(rd_star, 155.0);
    EXPECT_LE(rd_star, p.r_outter);
    EXPECT_DOUBLE_EQ(211.583, p.car_length);
    EXPECT_DOUBLE_EQ(53.453, p.car_width);
    EXPECT_DOUBLE_EQ(58.1855, p.car_center[0]);   // axis_x/2
    EXPECT_DOUBLE_EQ(0.0, p.car_center[1]);
    EXPECT_GT(p.step, 0.0);
    EXPECT_GT(p.k_slop, 0.0);
    EXPECT_GT(p.max_z, 0.0);
    EXPECT_DOUBLE_EQ(2.2, p.z_exp);
}

// Ship 8/8 camera coverage (mesh bake, retained from the previous round):
// with the real data_8 ship layout every one of the 8 cameras must have
// baked vertices with weight — the derived bowl grid must reach all cameras
// and no camera may be pushed behind the mesh reach by the new formula.
TEST(DeriveBowlParamsTest, ShipScale_All8CamerasSeeBowlWithWeight) {
    const BowlParams p = DeriveBowlParams(211.583, 53.453, 116.371);
    ASSERT_GE(p.r_outter, 150.0);
    // Real data_8 camera positions (t from examples/data_8/calibration.json),
    // each aimed at the ground at the data origin via the shared look-at
    // helper (synthetic stand-in for the real extrinsics).
    const double kCamXY[8][2] = {
        {96.71248, 0.0},        {75.55415, 25.65737},
        {17.36875, 25.65737},   {-40.81666, 25.65737},
        {75.55415, -25.65737},  {17.36875, -25.65737},
        {-40.81666, -25.65737}, {-91.59664, 0.0},
    };
    const double kCamZ[8] = {19.00911, 30.88980, 30.88980, 30.88980,
                             30.88980, 30.88980, 30.88980, 28.51366};
    std::vector<CameraIntrinsics> intrs(8, MakeIntrinsics());
    std::vector<CameraExtrinsics> extrs;
    for (int j = 0; j < 8; ++j) {
        const double r = std::hypot(kCamXY[j][0], kCamXY[j][1]);
        const double az = std::atan2(kCamXY[j][1], kCamXY[j][0]) * 180.0 / M_PI;
        extrs.push_back(test::MakeLookAtCamera(az, r, kCamZ[j], 0.0));
    }
    const BowlModel model(p, intrs, extrs);
    ASSERT_GT(model.vertex_count(), 0);
    for (int j = 0; j < 8; ++j) {
        int weighted = 0;
        for (int i = 0; i < model.vertex_count(); ++i) {
            if (model.baked_vertex(i).weight[j] > 1e-9) ++weighted;
        }
        EXPECT_GT(weighted, 0) << "camera " << j << " has no weighted vertices";
    }
}

// Grid topology must be CONSTANT across scales: r_outter/step == 63 (within
// ±1), so the square grid stays 126x126 (~16k vertices) for any vehicle —
// the previous unbounded-growth family (vertex counts growing with scale)
// is eliminated. Vertex budget re-pinned as well.
TEST(DeriveBowlParamsTest, GridTopology_Constant126x126AcrossScales) {
    for (double L : {5.6, 18.0, 211.583, 423.166}) {
        const BowlParams p = DeriveBowlParams(L, 2.2, L / 2.0);
        const double ratio = p.r_outter / p.step;
        EXPECT_NEAR(63.0, ratio, 1.0) << "L=" << L;
        const long rows = static_cast<long>(std::lround(ratio)) * 2;
        EXPECT_EQ(126, rows) << "L=" << L;
        const long max_vertices = (rows + 1) * (rows + 1);
        EXPECT_LE(max_vertices, 150000) << "L=" << L;
        EXPECT_GT(max_vertices, 0) << "L=" << L;
    }
}

// D1: car_center=(0,0) produces same vertex_count as before
TEST(BowlParamsTest, DefaultCarCenter_BackwardCompatVertexCount) {
    BowlParams p;
    auto model = BowlModel(p, DefaultIntrinsics(), DefaultBakeCameras());
    int count_default = model.vertex_count();

    // Explicit (0,0) should match default
    p.car_center = cv::Vec2d(0.0, 0.0);
    auto model2 = BowlModel(p, DefaultIntrinsics(), DefaultBakeCameras());
    EXPECT_EQ(count_default, model2.vertex_count());
}

// D1: ComputeRd offset — with car_center shifted, flat region moves with it
TEST(BowlParamsTest, ComputeRd_CarCenterShiftsFlatRegion) {
    BowlParams params;
    params.car_length = 2.0;     // car_l_half = 1.0
    params.car_width = 1.0;      // car_w_half = 0.5
    params.round_l = 0.5;
    params.car_center = cv::Vec2d(2.0, 0.0);

    // Point (3.0, 0.0): x - car_center[0] = 1.0 → |1.0| - (1.0 + 0.5) = -0.5 → dx=0.0
    // → inside flat region → rd = 0.0
    double rd = BowlModel::ComputeRd(3.0, 0.0, params);
    EXPECT_DOUBLE_EQ(0.0, rd);

    // Same point with default center (0,0): |3.0| - (1.0 + 0.5) = 1.5 → dx=1.5 → rd > 0
    BowlParams default_params;
    default_params.car_length = 2.0;
    default_params.car_width = 1.0;
    default_params.round_l = 0.5;
    double rd_default = BowlModel::ComputeRd(3.0, 0.0, default_params);
    EXPECT_GT(rd_default, 0.0);
}

// D1: Non-zero car_center shifts the flat region's centre
TEST(BowlModelTest, CarCenter_FlatRegionCenteredAtCarCenter) {
    BowlParams params = CarBowlCentered();
    params.car_length = 6.0;     // car_l_half = 3.0
    params.car_width = 4.0;      // car_w_half = 2.0
    params.round_l = 0.5;
    params.step = 0.2;
    params.car_center = cv::Vec2d(1.55, 0.0);

    const BowlModel model(params, DefaultIntrinsics(), DefaultBakeCameras());

    // Count vertices with x > car_center[0] vs x < car_center[0] inside the
    // flat centre region (z == 0). They should be roughly symmetric.
    int left_count = 0, right_count = 0;
    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& v = model.vertex(i);
        if (v[2] > 1e-9) continue;  // skip non-flat
        // Only count vertices within the flat rectangle bounds (relative to car_center)
        double rel_x = v[0] - params.car_center[0];
        double rel_y = v[1] - params.car_center[1];
        if (std::abs(rel_x) < params.car_length / 2.0 + params.round_l - 0.1 &&
            std::abs(rel_y) < params.car_width / 2.0 + params.round_l - 0.1) {
            if (rel_x < -0.01) left_count++;
            else if (rel_x > 0.01) right_count++;
        }
    }

    // The flat region should be roughly symmetric around car_center
    // (left and right counts within 20% of each other)
    int total = left_count + right_count;
    EXPECT_GT(total, 0) << "No flat-region vertices found straddling car_center";
    EXPECT_NEAR(left_count, right_count, total * 0.20)
        << "left=" << left_count << " right=" << right_count;
}

// D1: BlockBoat grid extent must be symmetric about car_center (not about origin).
// With car_center=(1.45,0), the grid vertices must span [cx-r_outter, cx+r_outter]
// so the front and back hemispheres have equal radius — currently the front
// radius is truncated to 11.15 (12.6-1.45) while the back reaches 14.05.
// Front-extent (φ≈0° relative to car_center, within step of x-axis) and
// back-extent (φ≈180°) must have equal max distance from car_center.
TEST(BowlModelTest, CarCenter_GridExtentSymmetricAboutCarCenter) {
    BowlParams params = CarBowlCentered();
    params.car_center = cv::Vec2d(1.45, 0.0);
    const BowlModel model(params, DefaultIntrinsics(), DefaultBakeCameras());
    ASSERT_GT(model.vertex_count(), 0);

    double min_x = std::numeric_limits<double>::max();
    double max_x = -std::numeric_limits<double>::max();

    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& v = model.vertex(i);
        min_x = std::min(min_x, v[0]);
        max_x = std::max(max_x, v[0]);
    }

    const double cx = 1.45;
    const double r_outter = 12.6;
    const double step = 0.2;

    // Front extent (positive x relative to car_center) must be r_outter.
    // Currently: max_x - cx = 12.6 - 1.45 = 11.15 — off by 1.45 (car_center shift).
    EXPECT_NEAR(r_outter, max_x - cx, step)
        << "front extent (max_x - cx) should be r_outter, got " << (max_x - cx);
    // Back extent (negative x relative to car_center) must also be r_outter.
    // Currently: cx - min_x = 1.45 - (-12.6) = 14.05 — too large.
    EXPECT_NEAR(r_outter, cx - min_x, step)
        << "back extent (cx - min_x) should be r_outter, got " << (cx - min_x);

    // Front/back symmetry: points near y≈car_center[1] along the x-axis:
    // max distance from car_center in front (φ≈0°) ≈ back (φ≈180°) within 1 step.
    // Use y within step of car_center[1] to isolate the fore-aft direction.
    double max_front_axial = 0.0, max_back_axial = 0.0;
    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& v = model.vertex(i);
        double rel_y = v[1] - params.car_center[1];
        if (std::abs(rel_y) > step) continue;  // near x-axis only
        double dx = v[0] - cx;
        double dist = std::abs(dx);
        if (dx > 0) {
            max_front_axial = std::max(max_front_axial, dist);
        } else if (dx < 0) {
            max_back_axial = std::max(max_back_axial, dist);
        }
    }
    EXPECT_GT(max_front_axial, 0.0) << "must have front-axial vertices";
    EXPECT_GT(max_back_axial, 0.0) << "must have back-axial vertices";
    // Currently: front ≈ 11.15, back ≈ 12.46 — asymmetric (difference ≈ 1.3)
    EXPECT_NEAR(max_front_axial, max_back_axial, step)
        << "front axial radius (" << max_front_axial
        << ") should ≈ back axial radius (" << max_back_axial << ")";
    // Both must be approximately r_outter
    EXPECT_NEAR(r_outter, max_front_axial, step);
    EXPECT_NEAR(r_outter, max_back_axial, step);
}

// D1: car_center=(0,0) must produce identical grid extent as before (regression guard).
// This duplicates the assertions of VerticesInXyBounds but with the explicit
// car_center=(0,0) case — the existing test already covers this with default params.
TEST(BowlModelTest, CarCenter_DefaultUnchanged) {
    const BowlParams p = CarBowlCentered();
    const BowlModel model(p, DefaultIntrinsics(), DefaultBakeCameras());
    ASSERT_GT(model.vertex_count(), 0);
    const double r_outter = p.r_outter;
    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& v = model.vertex(i);
        EXPECT_GE(v[0], -r_outter - 1e-9);
        EXPECT_LE(v[0], r_outter + 1e-9);
        EXPECT_GE(v[1], -r_outter - 1e-9);
        EXPECT_LE(v[1], r_outter + 1e-9);
    }
}

// ---------------------------------------------------------------------------
// BowlModel construction
// ---------------------------------------------------------------------------

TEST(BowlModelTest, ReturnsNonEmptyMesh) {
    const BowlModel model(CarBowlCentered(), DefaultIntrinsics(),
                          DefaultBakeCameras());
    EXPECT_GT(model.vertex_count(), 0);
    EXPECT_GT(model.triangle_count(), 0);
    // Construction invariant: one normal per vertex (kept from the removed
    // counts-consistency case, its only incremental assertion).
    EXPECT_EQ(model.vertex_count(), model.normal_count());
}

TEST(BowlModelTest, GridSizeMatchesParams) {
    const BowlParams p = CarBowlCentered();
    const BowlModel model(p, DefaultIntrinsics(), DefaultBakeCameras());
    // BlockBoat: rows = cols = lround(r_outter/step)*2 = lround(12.6/0.2)*2 = 126.
    // With no public rows()/cols() accessor, verify the same invariant through
    // the public geometry: the grid's distinct x columns span [-r_outter,
    // r_outter] and are spaced exactly `step` (127 columns = rows_ + 1).
    std::vector<double> xs;
    xs.reserve(model.vertex_count());
    for (int i = 0; i < model.vertex_count(); ++i) {
        xs.push_back(model.vertex(i)[0]);
    }
    std::sort(xs.begin(), xs.end());
    xs.erase(std::unique(xs.begin(), xs.end(),
                         [](double a, double b) { return std::abs(a - b) < 1e-9; }),
             xs.end());
    ASSERT_GE(xs.size(), 2u);
    EXPECT_EQ(127, static_cast<int>(xs.size()));   // rows_ + 1
    EXPECT_NEAR(p.step, xs[1] - xs[0], 1e-9);
    EXPECT_NEAR(2.0 * p.r_outter, xs.back() - xs.front(), 1e-9);
}

TEST(BowlModelTest, VerticesInXyBounds) {
    const BowlModel model(CarBowlCentered(), DefaultIntrinsics(),
                         DefaultBakeCameras());
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
    const BowlParams p = CarBowlCentered();
    const BowlModel model(p, DefaultIntrinsics(), DefaultBakeCameras());
    ASSERT_GT(model.vertex_count(), 0);
    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& v = model.vertex(i);
        EXPECT_GE(v[2], 0.0);
        EXPECT_LE(v[2], p.max_z + 1e-9);
    }
}

TEST(BowlModelTest, FlatCenterRegion) {
    BowlParams params = CarBowlCentered();
    params.car_length = 2.0;
    params.car_width = 1.0;
    params.round_l = 0.5;
    params.step = 0.2;
    params.r_outter = 12.6;
    double x_half = params.car_length / 2.0 + params.round_l; // 1.5
    double y_half = params.car_width / 2.0 + params.round_l;  // 1.0
    const BowlModel model(params, DefaultIntrinsics(), DefaultBakeCameras());
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
    // BlockBoat: z increases monotonically with rd (distance to car
    // rectangle). Group vertices by rd and verify z is non-decreasing.
    const BowlParams params = CarBowlCentered();
    const BowlModel model(params, DefaultIntrinsics(),
                         DefaultBakeCameras());
    ASSERT_GT(model.vertex_count(), 0);

    // Collect (rd, z) pairs, then sort by rd and check z monotonicity.
    std::vector<std::pair<double, double>> rd_z;
    rd_z.reserve(model.vertex_count());
    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& v = model.vertex(i);
        double rd = BowlModel::ComputeRd(v[0], v[1], params);
        rd_z.push_back({rd, v[2]});
    }
    std::sort(rd_z.begin(), rd_z.end());

    for (size_t i = 1; i < rd_z.size(); ++i) {
        if (rd_z[i].first > rd_z[i-1].first + 1e-12) {
            // Different rd bucket: z must be non-decreasing.
            EXPECT_GE(rd_z[i].second, rd_z[i-1].second - 1e-9)
                << "z not monotonic at rd " << rd_z[i].first;
        }
    }
}

TEST(BowlModelTest, NormalsAreUnitLength) {
    const BowlModel model(CarBowlCentered(), DefaultIntrinsics(),
                         DefaultBakeCameras());
    ASSERT_EQ(model.vertex_count(), model.normal_count());
    for (int i = 0; i < model.normal_count(); ++i) {
        const cv::Vec3d& n = model.normal(i);
        double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        EXPECT_NEAR(1.0, len, 1e-6);
    }
}

TEST(BowlModelTest, FlatCenterNormalsUpward) {
    BowlParams params = CarBowlCentered();
    params.car_length = 2.0;
    params.car_width = 1.0;
    params.round_l = 0.5;
    params.step = 0.2;
    params.r_outter = 12.6;
    double x_half = 1.5;
    double y_half = 1.0;
    const BowlModel model(params, DefaultIntrinsics(), DefaultBakeCameras());
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
    const BowlModel model(CarBowlCentered(), DefaultIntrinsics(),
                         DefaultBakeCameras());
    ASSERT_GT(model.triangle_count(), 0);
    for (int t = 0; t < model.triangle_count(); ++t) {
        for (int idx : model.triangle(t)) {
            EXPECT_GE(idx, 0);
            EXPECT_LT(idx, model.vertex_count());
        }
    }
}

TEST(BowlModelTest, DifferentCarSizeChangesFlatRegion) {
    BowlParams truck_params = CarBowlCentered();
    truck_params.car_length = 18.0;
    truck_params.car_width = 2.5;
    truck_params.step = 0.5;   // coarse for speed
    const BowlModel truck_model(truck_params, DefaultIntrinsics(),
                               DefaultBakeCameras());
    EXPECT_GT(truck_model.vertex_count(), 0);

    BowlParams default_params = CarBowlCentered();
    default_params.step = 0.5; // coarse for speed (same grid density as truck)
    const BowlModel default_model(default_params, DefaultIntrinsics(),
                                  DefaultBakeCameras());

    size_t truck_flat = CountFlatVertices(truck_model, 18.0 / 2.0 + 2.0, 2.5 / 2.0 + 2.0);
    size_t default_flat = CountFlatVertices(default_model, 5.6 / 2.0 + 2.0, 2.2 / 2.0 + 2.0);
    EXPECT_GT(truck_flat, default_flat);
}

TEST(BowlModelTest, MinimalParams_OneCellMesh) {
    BowlParams params;
    params.step = 12.6;    // step == r_outter -> rows = cols = lround(1)*2 = 2
    params.r_outter = 12.6;
    const BowlModel model(params, DefaultIntrinsics(), DefaultBakeCameras());
    EXPECT_GT(model.vertex_count(), 0);
    // must not crash; indices still valid
    for (int t = 0; t < model.triangle_count(); ++t) {
        for (int idx : model.triangle(t)) {
            EXPECT_LT(idx, model.vertex_count());
        }
    }
}

// ---------------------------------------------------------------------------
// Fine-grained index access: per-element queries (no container exposure).
// The former counts/vertex-bounds/unit-normal/triangle-index cases duplicated
// ReturnsNonEmptyMesh + VerticesInXyBounds + VerticesZInRange /
// NormalsAreUnitLength / TrianglesHaveValidIndices and were removed.
// ---------------------------------------------------------------------------

TEST(BowlModelTest, IndexAccess_WrappingConstructorCounts) {
    // Wrap-constructor model: counts reflect the wrapped data, triangles empty.
    BowlModel model({cv::Vec3d(0.0, 0.0, 10.0)}, {cv::Vec3d(0.0, 0.0, 1.0)},
                    {MakeIntrinsics()},
                    {CameraExtrinsics{cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)}});
    EXPECT_EQ(1, model.vertex_count());
    EXPECT_EQ(1, model.normal_count());
    EXPECT_EQ(0, model.triangle_count());
    EXPECT_DOUBLE_EQ(10.0, model.vertex(0)[2]);
    EXPECT_NEAR(1.0, model.normal(0)[2], 1e-9);
}

// ---------------------------------------------------------------------------
// Bake: per-vertex UV + blend weights (assertions kept verbatim from the
// former bake test file; only the construction form changed: mesh + bake
// fused into the BowlModel constructor).
// ---------------------------------------------------------------------------

TEST(BowlModelBakeTest, SingleCamera_UVMatchesProjection) {
    CameraIntrinsics intr = MakeIntrinsics();
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)};
    auto model = MakeCoarseBowl(5.0, {intr}, {extr});
    ASSERT_GT(model.vertex_count(), 0);

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

TEST(BowlModelBakeTest, N4_NumCameras) {
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    auto model = MakeCoarseBowl(12.6, intrs, extrs);
    EXPECT_EQ(4, model.baked_num_cameras());
}

TEST(BowlModelBakeTest, N4_WeightsSumToOne_WhenVisible) {
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    auto model = MakeCoarseBowl(12.6, intrs, extrs);

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
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    auto model = MakeCoarseBowl(12.6, intrs, extrs);

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
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    auto model = MakeCoarseBowl(12.6, intrs, extrs);

    // D3: Every vertex that a camera reports as invisible (by rect+circle
    // criteria in the bake) must carry weight 0 for that camera. Cameras
    // that ARE visible get non-zero weight from normal or D3 fallback
    // normalization. Compute visibility inline matching BakeVertices exact
    // logic to avoid any helper mismatch.
    bool found_invisible_pair = false;
    bool found_flat_vertex = false;
    for (int i = 0; i < model.vertex_count(); ++i) {
        if (model.vertex(i)[2] == 0.0) found_flat_vertex = true;
        for (int j = 0; j < 4; ++j) {
            double u_px, v_px;
            ProjectWorldToFishEye(model.vertex(i), intrs[j], extrs[j], u_px, v_px);
            bool visible_in_bake = u_px >= 0.0 && u_px < intrs[j].width
                                   && v_px >= 0.0 && v_px < intrs[j].height
                                   && IsInsideFisheyeCircle(intrs[j], u_px, v_px);
            if (!visible_in_bake) {
                found_invisible_pair = true;
                EXPECT_FLOAT_EQ(0.0f, model.baked_vertex(i).weight[j]);
            }
        }
    }
    EXPECT_TRUE(found_invisible_pair);
    EXPECT_TRUE(found_flat_vertex);
}

TEST(BowlModelBakeTest, N4_AllVerticesHaveAtLeastOneVisible) {
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    auto model = MakeCoarseBowl(12.6, intrs, extrs);

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
    auto intrs = std::vector<CameraIntrinsics>(8, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 45, 90, 135, 180, 225, 270, 315});
    auto model = MakeCoarseBowl(12.6, intrs, extrs);
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
    auto intrs = std::vector<CameraIntrinsics>(6, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 60, 120, 180, 240, 300});
    auto model = MakeCoarseBowl(12.6, intrs, extrs);
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
                    {cv::Vec3d(0.0, 0.0, 1.0), cv::Vec3d(0.0, 0.0, 1.0)},
                    {intr}, {extr});

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
    // D3: With IsForwardFacing removed, bowl-center vertices that project to
    // the image centre (degenerate r_xy=0 branch) become visible. To test an
    // all-invisible scenario, use a point whose projection falls outside the
    // image rectangle: P=(10,0,0.2) with identity cam at (0,0,5) gives
    // u≈1095 > 640 (for the 640x480 intrinsics).
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};
    cv::Vec3d invisible(10.0, 0.0, 0.2);
    BowlModel model({invisible}, {cv::Vec3d(0,0,1)}, {intr}, {extr});
    EXPECT_FLOAT_EQ(0.0f, model.baked_vertex(0).weight[0]);
    EXPECT_FLOAT_EQ(0.0f, model.baked_vertex(0).uv[0][0]);
    EXPECT_FLOAT_EQ(0.0f, model.baked_vertex(0).uv[0][1]);
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
    BowlModel model({cv::Vec3d(0.0, 0.0, 10.0)}, {cv::Vec3d(0.0, 0.0, 1.0)},
                    {intr}, {extr});

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

    BowlModel model({P}, {cv::Vec3d(0.0, 0.0, 1.0)}, {intr, intr}, {cam0, cam1});

    // Raw blend weights are symmetric (0.5 / 0.5); only the feather breaks the
    // tie: cam0's edge sample (feather ~0.625) must lose to cam1's centre one.
    EXPECT_LT(model.baked_vertex(0).weight[0], model.baked_vertex(0).weight[1]);
    EXPECT_LT(model.baked_vertex(0).weight[0], 0.5f);
    EXPECT_GT(model.baked_vertex(0).weight[1], 0.5f);
    // Normalization invariant is preserved.
    EXPECT_NEAR(1.0, model.baked_vertex(0).weight[0] + model.baked_vertex(0).weight[1], 1e-6);
}

// D1 + D2: car_center propagates through BakeVertices → ComputeBlendWeights pole.
// A non-zero car_center shifts the sector centre, changing weight distribution
// for the same world point. Only tests with visible points (P[x|y] > 3 for
// MakeSideCameras with radius=3m looking horizontally).
TEST(BowlModelBakeTest, CarCenter_ShiftsBlendWeightDistribution) {
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 90, 180, -90});

    // Point (5, 5, 1.1): visible to front (cam 0, az=0°) and left (cam 1, az=90°)
    // because P[0]=5 > camera_x=3 and P[1]=5 > camera_y=3.
    cv::Vec3d P(5.0, 5.0, 1.1);

    // Default car_center=(0,0): theta = atan2(5, 5) = 45° → on boundary → front+left ~0.5/0.5
    BowlModel model_default({P}, {cv::Vec3d(0,0,1)}, intrs, extrs);
    const auto& bv_default = model_default.baked_vertex(0);

    // With car_center=(3,0): same P → rel=(2,5) → theta = atan2(5, 2) ≈ 68.2°
    // Closer to left(90°) → left weight > front weight
    BowlModel model_shifted({P}, {cv::Vec3d(0,0,1)}, intrs, extrs,
                            cv::Vec2d(3.0, 0.0));
    const auto& bv_shifted = model_shifted.baked_vertex(0);

    // Both visible — weight sums to 1.0
    double sum_default = 0.0, sum_shifted = 0.0;
    for (int j = 0; j < 4; ++j) {
        sum_default += bv_default.weight[j];
        sum_shifted += bv_shifted.weight[j];
    }
    EXPECT_NEAR(1.0, sum_default, 1e-6);
    EXPECT_NEAR(1.0, sum_shifted, 1e-6);

    // Default: at boundary (45°) → front/left near equal
    EXPECT_GT(bv_default.weight[0], 0.4);
    EXPECT_GT(bv_default.weight[1], 0.4);

    // Shifted: pole moved to (3,0) → azimuth toward left(90°) → left weight dominates
    EXPECT_GT(bv_shifted.weight[1], bv_shifted.weight[0]);
}

TEST(BowlModelBakeTest, HasVisibleVertices_TrueWhenRimVisible) {
    // Bowl rim (z > 1) rises above the z=1 camera ring -> visible.
    auto intrs = std::vector<CameraIntrinsics>(4, MakeIntrinsics());
    auto extrs = MakeSideCameras({0, 180, 90, -90});
    auto model = MakeCoarseBowl(12.6, intrs, extrs);
    EXPECT_TRUE(model.HasVisibleVertices());
}

TEST(BowlModelBakeTest, HasVisibleVertices_FalseWhenAllInvisible) {
    // D3: With IsForwardFacing removed, bowl-centre vertices that project
    // to the image centre become visible. Use a truly invisible point:
    // P=(10,0,0.2), identity cam at (0,0,5), u≈1095 > 640.
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};
    cv::Vec3d invisible(10.0, 0.0, 0.2);
    BowlModel model({invisible}, {cv::Vec3d(0,0,1)}, {intr}, {extr});
    EXPECT_FALSE(model.HasVisibleVertices());
}

// ---------------------------------------------------------------------------
// D3: Fallback normalization — owner invisible but visible cameras exist
// ---------------------------------------------------------------------------

// D3: Owner invisible but neighbor visible → non-zero weights on visible set.
// Camera 0: identity rotation at (0,0,5) looking up. Point (-3,0,0.2):
// P_cam=(-3,0,-4.8) → θ≈2.58, u≈1095 > 640 → outside rect → invisible.
// Camera 1: lookAt at (6,0,2) toward origin. Point (-3,0,0.2) is in cam1's
// azimuth sector (180°) → raw_weight[1]=1.0, projects inside circle → visible.
TEST(BowlModelBakeTest, D3_FallbackNormalization_OwnerInvisibleNeighborVisible) {
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    CameraExtrinsics cam0{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};
    auto cam1 = avm_stitch::test::MakeLookAtCamera(0.0, 6.0, 2.0, 0.0);
    cv::Vec3d P(-3.0, 0.0, 0.2);

    BowlModel model({P}, {cv::Vec3d(0,0,1)}, {intr, intr}, {cam0, cam1});

    // Camera 0: owner, invisible (outside rect) → weight[0] stays 0
    // Camera 1: neighbor, visible → weight[1] > 0, sum == 1
    double sum = model.baked_vertex(0).weight[0] + model.baked_vertex(0).weight[1];
    EXPECT_NEAR(1.0, sum, 1e-6);
    EXPECT_FLOAT_EQ(0.0f, model.baked_vertex(0).weight[0]);
    EXPECT_GT(model.baked_vertex(0).weight[1], 0.0f);
}

// D3: No visible cameras at all → weights stay zero (physical blind zone).
TEST(BowlModelBakeTest, D3_FallbackNormalization_AllInvisible_WeightsStayZero) {
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    // Single camera above looking up: any point below is radiometrically
    // invisible (projects outside rect or circle).
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};
    cv::Vec3d P(10.0, 0.0, 0.2);  // u=920 > 640, outside rect
    BowlModel model({P}, {cv::Vec3d(0,0,1)}, {intr}, {extr});
    EXPECT_FLOAT_EQ(0.0f, model.baked_vertex(0).weight[0]);
}

// D3: IsForwardFacing no longer gates visibility — a z<0 point that projects
// inside the rect+circle is now visible (old IsForwardFacing would reject it).
// Identity camera at (0,0,5), point P=(0,0,3): P_cam=(0,0,-2), r_xy=0 →
// degenerate branch → u=cx=320, v=cy=240 → inside rect AND inside circle.
TEST(BowlModelBakeTest, D3_IsForwardFacingNoLongerGates) {
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};
    cv::Vec3d P(0.0, 0.0, 3.0);  // 2m below camera, z<0 in camera space

    BowlModel model({P}, {cv::Vec3d(0,0,1)}, {intr}, {extr});

    // Old code: IsForwardFacing rejects → weight = 0
    // New code: projection center (cx,cy) inside rect+circle → weight > 0
    EXPECT_GT(model.baked_vertex(0).weight[0], 0.0f);
}

// ---------------------------------------------------------------------------
// Black-seam regression tests: feather hard-gate + edge-factor visibility.
// ---------------------------------------------------------------------------

// Test A (core regression): A vertex seen by camera X at unsafe margin (< 40px,
// edge_factor=0), and by camera Y at image center (edge_factor=1.0). The vertex
// lies in both cameras' sector (both have identity rotation, same azimuth → raw
// weights 0.5/0.5). With the hard gate, camera X is invisible, so the full
// weight must go to camera Y.
TEST(BowlModelBakeTest, EdgeGate_UnsafeCameraInvisible_SafeCameraGetsFullWeight) {
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};

    // Build a point that camera 0 sees at r=230 (margin=10 < 40 → edge_factor=0)
    // and camera 1 sees at its image centre (margin=240 → edge_factor=1).
    // Both cameras are identity (look up) at different horizontal positions.
    const double r_u = std::tan(230.0 / 300.0);
    cv::Vec3d P(r_u * 5.0, 0.0, 10.0);

    // Camera 0: at (0,0,5), identity rotation → P projects at r=230, margin=10
    CameraExtrinsics cam0{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};
    // Camera 1: shifted to align P with its optical axis → P projects at centre
    CameraExtrinsics cam1{cv::Matx33d::eye(), cv::Vec3d(r_u * 5.0, 0.0, 5.0)};

    // Sanity: verify projections
    double u0, v0, u1, v1;
    ProjectWorldToFishEye(P, intr, cam0, u0, v0);
    ProjectWorldToFishEye(P, intr, cam1, u1, v1);
    ASSERT_NEAR(230.0, std::hypot(u0 - intr.cx, v0 - intr.cy), 1e-6);
    ASSERT_NEAR(0.0, std::hypot(u1 - intr.cx, v1 - intr.cy), 1e-6);

    BowlModel model({P}, {cv::Vec3d(0.0, 0.0, 1.0)}, {intr, intr}, {cam0, cam1});

    // Camera 0 is at margin=10 < 40 → edge_factor=0 → invisible → weight stays 0
    EXPECT_FLOAT_EQ(0.0f, model.baked_vertex(0).weight[0]);
    // Camera 1 is at centre → edge_factor=1 → visible → gets all weight
    EXPECT_NEAR(1.0f, model.baked_vertex(0).weight[1], 1e-6f);
    // Sum remains 1.0
    EXPECT_NEAR(1.0f, model.baked_vertex(0).weight[0] + model.baked_vertex(0).weight[1], 1e-6f);
}

// Test C: No safe camera — all cameras have margin < 40 → all edge_factor=0 →
// all invisible → weight_sum stays 0 (semantically honest zero-weight hole).
TEST(BowlModelBakeTest, EdgeGate_AllCamerasUnsafe_ZeroWeight) {
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};

    // Point projects to r=239 (margin=1 < 40) with a single identity camera
    // at (0,0,5).
    const double r_u = std::tan(239.0 / 300.0);
    cv::Vec3d P(r_u * 5.0, 0.0, 10.0);

    CameraExtrinsics cam{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};

    double u, v;
    ProjectWorldToFishEye(P, intr, cam, u, v);
    ASSERT_NEAR(239.0, std::hypot(u - intr.cx, v - intr.cy), 1e-6);

    BowlModel model({P}, {cv::Vec3d(0.0, 0.0, 1.0)}, {intr}, {cam});

    // Only camera is at margin=1 < 40 → edge_factor=0 → invisible → weight=0
    EXPECT_FLOAT_EQ(0.0f, model.baked_vertex(0).weight[0]);
}

