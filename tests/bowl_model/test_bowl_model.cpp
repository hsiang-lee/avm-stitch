#include <gtest/gtest.h>

#include "bowl_model/bowl_model.hpp"
#include "camera_model/camera.hpp"
#include "camera_model/camera_extrinsics.hpp"
#include "camera_model/kb_camera_model.hpp"

#include "test_support/stitch_test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace avm_stitch;

using avm_stitch::test::DefaultBakeCameras;
using avm_stitch::test::DefaultIntrinsics;
using avm_stitch::test::MakeCoarseBowl;
using avm_stitch::test::MakeCoarseBowlParams;
using avm_stitch::test::MakeDefaultKbCameras;
using avm_stitch::test::MakeRealIntrinsics;
using avm_stitch::test::MakeSideCameras;
using avm_stitch::test::WrapKbCameras;

namespace {

size_t CountFlatVertices(const BowlModel& model, double flat_x, double flat_y) {
    size_t count = 0;
    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& v = model.vertex(i);
        if (std::abs(v[0]) < flat_x - 1e-9 && std::abs(v[1]) < flat_y - 1e-9) {
            count++;
        }
    }
    return count;
}

bool IsVertexVisibleToCamera(const cv::Vec3d& P, const KbParams& intr,
                             const CameraExtrinsics& extr) {
    const KbCameraModel rep{intr};
    const cv::Vec2d uv = rep.ProjectCameraPoint(extr.ToCamera(P));
    return uv[0] >= 0.0 && uv[0] < rep.width() && uv[1] >= 0.0 && uv[1] < rep.height() && rep.InsideFisheyeCircle(uv[0], uv[1]);
}

bool IsVertexBakedVisible(const cv::Vec3d& P, const KbParams& intr,
                          const CameraExtrinsics& extr) {
    const KbCameraModel rep{intr};
    const cv::Vec2d uv = rep.ProjectCameraPoint(extr.ToCamera(P));
    return IsVertexVisibleToCamera(P, intr, extr) && rep.EdgeFeather(uv[0], uv[1]) > 0.0;
}

BowlParams CarBowlCentered() {
    const BowlParams derived = BowlParams::ForCarSize(5.6, 2.2, 2.9);
    return BowlParams(5.6, 2.2, derived.FlatPlatformRadius(), derived.GridStep(),
                      derived.OuterRadius(), derived.SlopeCoefficient(),
                      derived.HeightExponent(), derived.MaxHeight(), cv::Vec2d(0.0, 0.0));
}

BowlParams TallBowlParams() {
    return BowlParams(2.0, 1.0, 0.5, 0.1, 6.5, 3.2, 1.0, 15.0,
                      cv::Vec2d(0.0, 0.0));
}

} // namespace

TEST(BowlModelTest, CarCenter_FlatRegionCenteredAtCarCenter) {
    const BowlParams derived = BowlParams::ForCarSize(5.6, 2.2, 2.9);
    const BowlParams params(6.0, 4.0, 0.5, 0.2,
                            derived.OuterRadius(), derived.SlopeCoefficient(),
                            derived.HeightExponent(), derived.MaxHeight(),
                            cv::Vec2d(1.55, 0.0));
    const double flat_x = 6.0 / 2.0 + 0.5;
    const double flat_y = 4.0 / 2.0 + 0.5;

    const BowlModel model(params, MakeDefaultKbCameras());

    int left_count = 0, right_count = 0;
    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& v = model.vertex(i);
        if (v[2] > 1e-9)
            continue;
        double rel_x = v[0] - params.CarCenter()[0];
        double rel_y = v[1] - params.CarCenter()[1];
        if (std::abs(rel_x) < flat_x - 0.1 && std::abs(rel_y) < flat_y - 0.1) {
            if (rel_x < -0.01)
                left_count++;
            else if (rel_x > 0.01)
                right_count++;
        }
    }

    int total = left_count + right_count;
    EXPECT_GT(total, 0) << "No flat-region vertices found straddling car_center";
    EXPECT_NEAR(left_count, right_count, total * 0.20)
        << "left=" << left_count << " right=" << right_count;
}

TEST(BowlModelTest, CarCenter_GridExtentSymmetricAboutCarCenter) {
    const BowlParams derived = BowlParams::ForCarSize(5.6, 2.2, 2.9);
    const BowlParams params(5.6, 2.2, derived.FlatPlatformRadius(), derived.GridStep(),
                            derived.OuterRadius(), derived.SlopeCoefficient(),
                            derived.HeightExponent(), derived.MaxHeight(),
                            cv::Vec2d(1.45, 0.0));
    const BowlModel model(params, MakeDefaultKbCameras());
    ASSERT_GT(model.vertex_count(), 0);

    double min_x = std::numeric_limits<double>::max();
    double max_x = -std::numeric_limits<double>::max();

    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& v = model.vertex(i);
        min_x = std::min(min_x, v[0]);
        max_x = std::max(max_x, v[0]);
    }

    const double cx = 1.45;
    const double outer_radius = 12.6;
    const double step = 0.2;

    EXPECT_NEAR(outer_radius, max_x - cx, step)
        << "front extent (max_x - cx) should be outer_radius, got " << (max_x - cx);
    EXPECT_NEAR(outer_radius, cx - min_x, step)
        << "back extent (cx - min_x) should be outer_radius, got " << (cx - min_x);

    double max_front_axial = 0.0, max_back_axial = 0.0;
    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& v = model.vertex(i);
        double rel_y = v[1] - params.CarCenter()[1];
        if (std::abs(rel_y) > step)
            continue;
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
    EXPECT_NEAR(max_front_axial, max_back_axial, step)
        << "front axial radius (" << max_front_axial
        << ") should ≈ back axial radius (" << max_back_axial << ")";
    EXPECT_NEAR(outer_radius, max_front_axial, step);
    EXPECT_NEAR(outer_radius, max_back_axial, step);
}

TEST(BowlModelTest, CarCenter_DefaultUnchanged) {
    const BowlParams p = CarBowlCentered();
    const BowlModel model(p, MakeDefaultKbCameras());
    ASSERT_GT(model.vertex_count(), 0);
    const double outer_radius = p.OuterRadius();
    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& v = model.vertex(i);
        EXPECT_GE(v[0], -outer_radius - 1e-9);
        EXPECT_LE(v[0], outer_radius + 1e-9);
        EXPECT_GE(v[1], -outer_radius - 1e-9);
        EXPECT_LE(v[1], outer_radius + 1e-9);
    }
}

TEST(BowlModelTest, ReturnsNonEmptyMesh) {
    const BowlModel model(CarBowlCentered(), MakeDefaultKbCameras());
    EXPECT_GT(model.vertex_count(), 0);
    EXPECT_GT(model.triangle_count(), 0);
    EXPECT_EQ(model.vertex_count(), model.normal_count());
}

TEST(BowlModelTest, GridSizeMatchesParams) {
    const BowlParams p = CarBowlCentered();
    const BowlModel model(p, MakeDefaultKbCameras());
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
    EXPECT_EQ(127, static_cast<int>(xs.size()));
    EXPECT_NEAR(p.GridStep(), xs[1] - xs[0], 1e-9);
    EXPECT_NEAR(2.0 * p.OuterRadius(), xs.back() - xs.front(), 1e-9);
}

TEST(BowlModelTest, VerticesInXyBounds) {
    const BowlModel model(CarBowlCentered(), MakeDefaultKbCameras());
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
    const BowlModel model(p, MakeDefaultKbCameras());
    ASSERT_GT(model.vertex_count(), 0);
    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& v = model.vertex(i);
        EXPECT_GE(v[2], 0.0);
        EXPECT_LE(v[2], p.MaxHeight() + 1e-9);
    }
}

TEST(BowlModelTest, FlatCenterRegion) {
    const BowlParams derived = BowlParams::ForCarSize(5.6, 2.2, 2.9);
    const BowlParams params(2.0, 1.0, 0.5, 0.2, 12.6,
                            derived.SlopeCoefficient(), derived.HeightExponent(), derived.MaxHeight(),
                            cv::Vec2d(0.0, 0.0));
    const double flat_x = 2.0 / 2.0 + 0.5;
    const double flat_y = 1.0 / 2.0 + 0.5;
    const BowlModel model(params, MakeDefaultKbCameras());
    bool found_flat = false;
    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& v = model.vertex(i);
        if (std::abs(v[0]) < flat_x - 1e-9 && std::abs(v[1]) < flat_y - 1e-9) {
            found_flat = true;
            EXPECT_DOUBLE_EQ(0.0, v[2]);
        }
    }
    EXPECT_TRUE(found_flat);
}

TEST(BowlModelTest, EdgesMonotonic) {
    const BowlParams params = CarBowlCentered();
    const BowlModel model(params, MakeDefaultKbCameras());
    ASSERT_GT(model.vertex_count(), 0);

    std::vector<std::pair<double, double>> rd_z;
    rd_z.reserve(model.vertex_count());
    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& v = model.vertex(i);
        double rd = params.DistanceFromFlatPlatform(v[0], v[1]);
        rd_z.push_back({rd, v[2]});
    }
    std::sort(rd_z.begin(), rd_z.end());

    for (size_t i = 1; i < rd_z.size(); ++i) {
        if (rd_z[i].first > rd_z[i - 1].first + 1e-12) {
            EXPECT_GE(rd_z[i].second, rd_z[i - 1].second - 1e-9)
                << "z not monotonic at rd " << rd_z[i].first;
        }
    }
}

TEST(BowlModelTest, NormalsAreUnitLength) {
    const BowlModel model(CarBowlCentered(), MakeDefaultKbCameras());
    ASSERT_EQ(model.vertex_count(), model.normal_count());
    for (int i = 0; i < model.normal_count(); ++i) {
        const cv::Vec3d& n = model.normal(i);
        double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        EXPECT_NEAR(1.0, len, 1e-6);
    }
}

TEST(BowlModelTest, FlatCenterNormalsUpward) {
    const BowlParams derived = BowlParams::ForCarSize(5.6, 2.2, 2.9);
    const BowlParams params(2.0, 1.0, 0.5, 0.2, 12.6,
                            derived.SlopeCoefficient(), derived.HeightExponent(), derived.MaxHeight(),
                            cv::Vec2d(0.0, 0.0));
    double flat_x = 2.0 / 2.0 + 0.5;
    double flat_y = 1.0 / 2.0 + 0.5;
    const BowlModel model(params, MakeDefaultKbCameras());
    bool found_flat = false;
    for (int k = 0; k < model.vertex_count(); ++k) {
        const cv::Vec3d& v = model.vertex(k);
        if (std::abs(v[0]) < flat_x - 1e-9 && std::abs(v[1]) < flat_y - 1e-9) {
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
    const BowlModel model(CarBowlCentered(), MakeDefaultKbCameras());
    ASSERT_GT(model.triangle_count(), 0);
    for (int t = 0; t < model.triangle_count(); ++t) {
        for (int idx : model.triangle(t)) {
            EXPECT_GE(idx, 0);
            EXPECT_LT(idx, model.vertex_count());
        }
    }
}

TEST(BowlModelTest, DifferentCarSizeChangesFlatRegion) {
    const BowlParams derived = BowlParams::ForCarSize(5.6, 2.2, 2.9);
    const BowlParams truck_params(18.0, 2.5, derived.FlatPlatformRadius(), 0.5,
                                  derived.OuterRadius(), derived.SlopeCoefficient(),
                                  derived.HeightExponent(), derived.MaxHeight(),
                                  cv::Vec2d(0.0, 0.0));
    const BowlModel truck_model(truck_params, MakeDefaultKbCameras());
    EXPECT_GT(truck_model.vertex_count(), 0);

    const BowlParams default_params(5.6, 2.2, derived.FlatPlatformRadius(), 0.5,
                                    derived.OuterRadius(), derived.SlopeCoefficient(),
                                    derived.HeightExponent(), derived.MaxHeight(),
                                    cv::Vec2d(0.0, 0.0));
    const BowlModel default_model(default_params, MakeDefaultKbCameras());

    size_t truck_flat = CountFlatVertices(truck_model, 18.0 / 2.0 + 2.0, 2.5 / 2.0 + 2.0);
    size_t default_flat = CountFlatVertices(default_model, 5.6 / 2.0 + 2.0, 2.2 / 2.0 + 2.0);
    EXPECT_GT(truck_flat, default_flat);
}

TEST(BowlModelTest, MinimalParams_OneCellMesh) {
    const BowlParams params(0.0, 0.0, 0.0, 12.6, 12.6, 0.0, 0.0, 0.0);
    const BowlModel model(params, MakeDefaultKbCameras());
    EXPECT_GT(model.vertex_count(), 0);
    for (int t = 0; t < model.triangle_count(); ++t) {
        for (int idx : model.triangle(t)) {
            EXPECT_LT(idx, model.vertex_count());
        }
    }
}

TEST(BowlModelTest, BakeVertices_OneBakedPerMeshVertex) {
    auto intrs = std::vector<KbParams>(1, MakeRealIntrinsics());
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};
    auto model = MakeCoarseBowl(5.0, intrs, {extr});
    ASSERT_GT(model.vertex_count(), 0);
    ASSERT_EQ(1, model.baked_num_cameras());

    int seen_visible = 0;
    for (int i = 0; i < model.vertex_count(); ++i) {
        const BowlModel::BakedVertex& baked_vertex = model.baked_vertex(i);
        if (baked_vertex.weight[0] > 1e-9f)
            ++seen_visible;
        EXPECT_GE(baked_vertex.uv[0][0], 0.0f);
        EXPECT_LE(baked_vertex.uv[0][0], 1.0f);
        EXPECT_GE(baked_vertex.uv[0][1], 0.0f);
        EXPECT_LE(baked_vertex.uv[0][1], 1.0f);
    }
    EXPECT_GT(seen_visible, 0);
}

TEST(BowlModelBakeTest, SingleCamera_UVMatchesProjection) {
    KbParams intr = MakeRealIntrinsics();
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)};
    auto model = MakeCoarseBowl(5.0, {intr}, {extr});
    ASSERT_GT(model.vertex_count(), 0);

    ASSERT_EQ(1, model.baked_num_cameras());

    int checked = 0;
    const KbCameraModel rep{intr};
    for (int i = 0; i < model.vertex_count(); ++i) {
        if (!IsVertexVisibleToCamera(model.vertex(i), intr, extr))
            continue;
        ++checked;
        const cv::Vec2d uv = rep.ProjectCameraPoint(extr.ToCamera(model.vertex(i)));
        EXPECT_NEAR(model.baked_vertex(i).uv[0][0], uv[0] / rep.width(), 1e-6);
        EXPECT_NEAR(model.baked_vertex(i).uv[0][1], uv[1] / rep.height(), 1e-6);
        EXPECT_NEAR(model.baked_vertex(i).weight[0], 1.0, 1e-6);
    }
    EXPECT_GT(checked, 0);
}

class BowlModelBakeParamTest : public ::testing::TestWithParam<int> {};

TEST_P(BowlModelBakeParamTest, BakedSlots_MatchCameraCount) {
    const int n = GetParam();
    std::vector<double> azimuths;
    azimuths.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i)
        azimuths.push_back(i * 360.0 / n);

    auto intrs = std::vector<KbParams>(n, MakeRealIntrinsics());
    auto extrs = MakeSideCameras(azimuths);
    const BowlModel model = MakeCoarseBowl(12.6, intrs, extrs);

    ASSERT_EQ(n, model.baked_num_cameras());
    ASSERT_GT(model.vertex_count(), 0);

    bool last_slot_used = false;
    int visible_vertices = 0;
    for (int i = 0; i < model.vertex_count(); ++i) {
        const BowlModel::BakedVertex& baked_vertex = model.baked_vertex(i);
        EXPECT_EQ(static_cast<size_t>(n), baked_vertex.uv.size());
        EXPECT_EQ(static_cast<size_t>(n), baked_vertex.weight.size());
        double sum = 0.0;
        for (int j = 0; j < n; ++j) {
            sum += baked_vertex.weight[j];
            EXPECT_GE(baked_vertex.weight[j], 0.0f);
            EXPECT_GE(baked_vertex.uv[j][0], 0.0f);
            EXPECT_LE(baked_vertex.uv[j][0], 1.0f);
            EXPECT_GE(baked_vertex.uv[j][1], 0.0f);
            EXPECT_LE(baked_vertex.uv[j][1], 1.0f);
        }
        if (sum > 1e-9) {
            ++visible_vertices;
            EXPECT_NEAR(1.0, sum, 1e-6);
        }
        if (baked_vertex.weight[n - 1] > 0.0f)
            last_slot_used = true;
    }
    EXPECT_GT(visible_vertices, 0);
    EXPECT_TRUE(last_slot_used);
}

INSTANTIATE_TEST_SUITE_P(
    BowlModelBake, BowlModelBakeParamTest,
    ::testing::Values(4, 6, 8),
    [](const ::testing::TestParamInfo<int>& info) {
        return "N" + std::to_string(info.param);
    });

TEST(BowlModelBakeTest, BakedVertex_SizedByCameraCount) {
    auto intrs1 = std::vector<KbParams>(1, MakeRealIntrinsics());
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};
    const BowlModel model1 = MakeCoarseBowl(5.0, intrs1, {extr});
    ASSERT_EQ(1, model1.baked_num_cameras());
    ASSERT_GT(model1.vertex_count(), 0);
    const BowlModel::BakedVertex& bv1 = model1.baked_vertex(0);
    EXPECT_EQ(1u, bv1.uv.size());
    EXPECT_EQ(1u, bv1.weight.size());
}

TEST(BowlModelBakeTest, N4_WeightsSumToOne_WhenVisible) {
    auto intrs = DefaultIntrinsics();
    auto extrs = DefaultBakeCameras();
    auto model = MakeCoarseBowl(12.6, intrs, extrs);

    int visible_vertices = 0;
    for (int i = 0; i < model.vertex_count(); ++i) {
        const BowlModel::BakedVertex& baked_vertex = model.baked_vertex(i);
        double sum = 0.0;
        for (int j = 0; j < 4; ++j)
            sum += baked_vertex.weight[j];
        if (sum > 1e-9) {
            ++visible_vertices;
            EXPECT_NEAR(1.0, sum, 1e-6);
        }
    }
    EXPECT_GT(visible_vertices, 0);
}

TEST(BowlModelBakeTest, N4_UVInNormalizedRange) {
    auto intrs = DefaultIntrinsics();
    auto extrs = DefaultBakeCameras();
    auto model = MakeCoarseBowl(12.6, intrs, extrs);

    for (int i = 0; i < model.vertex_count(); ++i) {
        const BowlModel::BakedVertex& baked_vertex = model.baked_vertex(i);
        for (int j = 0; j < 4; ++j) {
            EXPECT_GE(baked_vertex.uv[j][0], 0.0f);
            EXPECT_LE(baked_vertex.uv[j][0], 1.0f);
            EXPECT_GE(baked_vertex.uv[j][1], 0.0f);
            EXPECT_LE(baked_vertex.uv[j][1], 1.0f);
        }
    }
}

TEST(BowlModelBakeTest, N4_InvisibleCameraHasZeroWeight) {
    auto intrs = DefaultIntrinsics();
    auto extrs = DefaultBakeCameras();
    auto model = MakeCoarseBowl(12.6, intrs, extrs);

    bool found_invisible_pair = false;
    bool found_flat_vertex = false;
    for (int i = 0; i < model.vertex_count(); ++i) {
        if (model.vertex(i)[2] == 0.0)
            found_flat_vertex = true;
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
    auto intrs = DefaultIntrinsics();
    auto extrs = DefaultBakeCameras();
    auto model = MakeCoarseBowl(12.6, intrs, extrs);

    int visible_vertices = 0;
    int invisible_vertices = 0;
    for (int i = 0; i < model.vertex_count(); ++i) {
        const BowlModel::BakedVertex& baked_vertex = model.baked_vertex(i);
        double sum = 0.0;
        for (int j = 0; j < 4; ++j)
            sum += baked_vertex.weight[j];
        if (sum > 1e-9)
            ++visible_vertices;
        else
            ++invisible_vertices;
    }
    EXPECT_GT(visible_vertices, 0);
    EXPECT_GT(invisible_vertices, 0);
}

TEST(BowlModelBakeTest, PointInRectButOutsideFisheyeCircle_Invisible) {
    KbParams intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};

    const BowlModel model(TallBowlParams(), WrapKbCameras({intr}, {extr}));
    ASSERT_GT(model.vertex_count(), 0);

    int inside_i = -1, outside_i = -1;
    double best_rpx = -1.0;
    const KbCameraModel rep{intr};
    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& P = model.vertex(i);
        const cv::Vec2d uv = rep.ProjectCameraPoint(extr.ToCamera(P));
        if (!(uv[0] >= 0.0 && uv[0] < rep.width() && uv[1] >= 0.0 && uv[1] < rep.height())) {
            continue;
        }
        const double r_px = std::hypot(uv[0] - rep.Cx(), uv[1] - rep.Cy());
        if (rep.InsideFisheyeCircle(uv[0], uv[1])) {
            if (rep.FisheyeCircleRadius() - r_px >= 40.0 && r_px > best_rpx) {
                best_rpx = r_px;
                inside_i = i;
            }
        } else if (outside_i < 0) {
            outside_i = i;
        }
    }
    ASSERT_GE(inside_i, 0)
        << "no vertex projects inside rect AND inside the circle with safe margin";
    ASSERT_GE(outside_i, 0)
        << "no vertex projects inside rect but OUTSIDE the circle";

    const cv::Vec2d uv_inside =
        rep.ProjectCameraPoint(extr.ToCamera(model.vertex(inside_i)));
    EXPECT_NEAR(1.0, model.baked_vertex(inside_i).weight[0], 1e-6);
    EXPECT_NEAR(uv_inside[0] / rep.width(), model.baked_vertex(inside_i).uv[0][0], 1e-6);
    EXPECT_NEAR(uv_inside[1] / rep.height(), model.baked_vertex(inside_i).uv[0][1], 1e-6);

    EXPECT_FLOAT_EQ(0.0f, model.baked_vertex(outside_i).weight[0]);
    EXPECT_FLOAT_EQ(0.0f, model.baked_vertex(outside_i).uv[0][0]);
    EXPECT_FLOAT_EQ(0.0f, model.baked_vertex(outside_i).uv[0][1]);
}

TEST(BowlModelBakeTest, AllInvisible_WeightsStayZero_HasVisibleVerticesFalse) {
    KbParams intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(30.0, 0.0, 5.0)};
    const BowlModel model(MakeCoarseBowlParams(12.6), WrapKbCameras({intr}, {extr}));
    ASSERT_GT(model.vertex_count(), 0);

    for (int i = 0; i < model.vertex_count(); ++i) {
        const BowlModel::BakedVertex& baked_vertex = model.baked_vertex(i);
        EXPECT_FLOAT_EQ(0.0f, baked_vertex.weight[0]) << "index " << i;
        EXPECT_FLOAT_EQ(0.0f, baked_vertex.uv[0][0]) << "index " << i;
        EXPECT_FLOAT_EQ(0.0f, baked_vertex.uv[0][1]) << "index " << i;
    }
    EXPECT_FALSE(model.HasVisibleVertices());
}

TEST(BowlModelBakeTest, SingleCamera_VisibleVerticesNormalizeToWeightOne) {
    KbParams intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};

    const BowlModel model(TallBowlParams(), WrapKbCameras({intr}, {extr}));
    ASSERT_GT(model.vertex_count(), 0);

    int visible = 0;
    for (int i = 0; i < model.vertex_count(); ++i) {
        const float w = model.baked_vertex(i).weight[0];
        if (w > 1e-9f) {
            ++visible;
            EXPECT_NEAR(1.0, w, 1e-6) << "index " << i;
        }
    }
    EXPECT_GT(visible, 0);
}

TEST(BowlModelBakeTest, EdgeGate_UnsafeCameraZero_SafeCameraGetsFullWeight) {
    KbParams intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    CameraExtrinsics cam0{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};

    const BowlModel scan(TallBowlParams(), WrapKbCameras({intr}, {cam0}));
    ASSERT_GT(scan.vertex_count(), 0);
    int target = -1;
    const KbCameraModel rep{intr};
    for (int i = 0; i < scan.vertex_count(); ++i) {
        const cv::Vec3d& P = scan.vertex(i);
        const cv::Vec2d uv = rep.ProjectCameraPoint(cam0.ToCamera(P));
        if (!(uv[0] >= 0.0 && uv[0] < rep.width() && uv[1] >= 0.0 && uv[1] < rep.height()))
            continue;
        const double r_px = std::hypot(uv[0] - rep.Cx(), uv[1] - rep.Cy());
        if (rep.InsideFisheyeCircle(uv[0], uv[1]) && rep.FisheyeCircleRadius() - r_px < 40.0) {
            target = i;
            break;
        }
    }
    ASSERT_GE(target, 0) << "no vertex in cam0's unsafe margin band";

    const cv::Vec3d& P = scan.vertex(target);
    CameraExtrinsics cam1{cv::Matx33d::eye(), cv::Vec3d(P[0], P[1], 5.0)};
    const BowlModel model(TallBowlParams(), WrapKbCameras({intr, intr}, {cam0, cam1}));

    const cv::Vec2d uv0 = rep.ProjectCameraPoint(cam0.ToCamera(P));
    const cv::Vec2d uv1 = rep.ProjectCameraPoint(cam1.ToCamera(P));
    ASSERT_LT(std::hypot(uv0[0] - rep.Cx(), uv0[1] - rep.Cy()), 240.0);
    ASSERT_LT(rep.FisheyeCircleRadius() - std::hypot(uv0[0] - rep.Cx(), uv0[1] - rep.Cy()), 40.0);
    ASSERT_LT(std::hypot(uv1[0] - rep.Cx(), uv1[1] - rep.Cy()), 1e-3);

    const auto& baked_vertex = model.baked_vertex(target);
    EXPECT_FLOAT_EQ(0.0f, baked_vertex.weight[0]);
    EXPECT_NEAR(1.0f, baked_vertex.weight[1], 1e-6f);
    EXPECT_NEAR(1.0, baked_vertex.weight[0] + baked_vertex.weight[1], 1e-6);
}

TEST(BowlModelBakeTest, CarCenter_ShiftsBlendWeightDistribution) {
    auto intrs = DefaultIntrinsics();
    auto extrs = MakeSideCameras({0, 90, 180, -90});
    auto cams = WrapKbCameras(intrs, extrs);

    const BowlParams base = MakeCoarseBowlParams(12.6);
    const BowlParams shifted(5.6, 2.2, base.FlatPlatformRadius(), 1.0, 12.6,
                             base.SlopeCoefficient(), base.HeightExponent(), base.MaxHeight(),
                             cv::Vec2d(3.0, 0.0));
    const BowlModel def(base, cams);
    const BowlModel sh(shifted, cams);

    auto find_world_diag = [&](const BowlModel& m) {
        for (int i = 0; i < m.vertex_count(); ++i) {
            const cv::Vec3d& P = m.vertex(i);
            if (!(P[0] > 3.0 && P[1] > 3.0 && std::abs(P[0] - P[1]) < 1.0))
                continue;
            if (IsVertexVisibleToCamera(P, intrs[0], extrs[0]) && IsVertexVisibleToCamera(P, intrs[1], extrs[1])) {
                return i;
            }
        }
        return -1;
    };

    const int di_def = find_world_diag(def);
    ASSERT_GE(di_def, 0) << "no world-diagonal vertex visible to front+left in centred bowl";

    const BowlModel::BakedVertex& bv_def = def.baked_vertex(di_def);
    double sum_def = 0.0;
    for (int j = 0; j < 4; ++j)
        sum_def += bv_def.weight[j];
    EXPECT_GT(bv_def.weight[0], 0.4);
    EXPECT_GT(bv_def.weight[1], 0.4);
    EXPECT_NEAR(1.0, sum_def, 1e-6);

    const int di_sh = find_world_diag(sh);
    ASSERT_GE(di_sh, 0) << "no world-diagonal vertex visible to front+left in shifted bowl";

    const BowlModel::BakedVertex& bv_sh = sh.baked_vertex(di_sh);
    double sum_sh = 0.0;
    for (int j = 0; j < 4; ++j)
        sum_sh += bv_sh.weight[j];
    EXPECT_GT(bv_sh.weight[1], bv_sh.weight[0]);
    EXPECT_NEAR(1.0, sum_sh, 1e-6);
}

TEST(BowlModelBakeTest, HasVisibleVertices_TrueWhenRimVisible) {
    auto intrs = DefaultIntrinsics();
    auto extrs = DefaultBakeCameras();
    auto model = MakeCoarseBowl(12.6, intrs, extrs);
    EXPECT_TRUE(model.HasVisibleVertices());
}

TEST(BowlModelBakeTest, D3_FallbackNormalization_OwnerInvisibleNeighborVisible) {
    KbParams intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    CameraExtrinsics cam0{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};
    auto cam1 = avm_stitch::test::MakeLookAtCamera(0.0, 6.0, 2.0, 0.0);

    const BowlModel model(MakeCoarseBowlParams(12.6),
                          WrapKbCameras({intr, intr}, {cam0, cam1}));
    ASSERT_GT(model.vertex_count(), 0);

    int target = -1;
    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& P = model.vertex(i);
        if (!IsVertexVisibleToCamera(P, intr, cam0) && IsVertexBakedVisible(P, intr, cam1)) {
            target = i;
            break;
        }
    }
    ASSERT_GE(target, 0) << "no vertex that cam0 misses but cam1 bakes visible";

    const BowlModel::BakedVertex& baked_vertex = model.baked_vertex(target);
    double sum = baked_vertex.weight[0] + baked_vertex.weight[1];
    EXPECT_NEAR(1.0, sum, 1e-6);
    EXPECT_FLOAT_EQ(0.0f, baked_vertex.weight[0]);
    EXPECT_GT(baked_vertex.weight[1], 0.0f);
}

TEST(BowlModelBakeTest, BelowCameraVertexStillGetsWeight) {
    KbParams intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};

    const BowlModel model(MakeCoarseBowlParams(5.0), WrapKbCameras({intr}, {extr}));
    ASSERT_GT(model.vertex_count(), 0);

    int target = -1;
    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& P = model.vertex(i);
        if (P[2] >= 5.0)
            continue;
        const cv::Vec3d P_cam = extr.ToCamera(P);
        const double r_xy = std::hypot(P_cam[0], P_cam[1]);
        if (r_xy < kEps && IsVertexVisibleToCamera(P, intr, extr)) {
            target = i;
            break;
        }
    }
    ASSERT_GE(target, 0) << "no degenerate centre vertex below the camera";

    EXPECT_GT(model.baked_vertex(target).weight[0], 0.0f);
}

TEST(BowlModelBakeTest, EdgeGate_AllCamerasUnsafe_ZeroWeight) {
    KbParams intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    CameraExtrinsics cam{cv::Matx33d::eye(), cv::Vec3d(0, 0, 5)};

    const BowlModel model(TallBowlParams(), WrapKbCameras({intr}, {cam}));
    ASSERT_GT(model.vertex_count(), 0);

    int target = -1;
    const KbCameraModel rep{intr};
    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& P = model.vertex(i);
        const cv::Vec2d uv = rep.ProjectCameraPoint(cam.ToCamera(P));
        if (!(uv[0] >= 0.0 && uv[0] < rep.width() && uv[1] >= 0.0 && uv[1] < rep.height()))
            continue;
        const double r_px = std::hypot(uv[0] - rep.Cx(), uv[1] - rep.Cy());
        if (rep.InsideFisheyeCircle(uv[0], uv[1]) && rep.FisheyeCircleRadius() - r_px < 40.0) {
            target = i;
            break;
        }
    }
    ASSERT_GE(target, 0) << "no vertex in the unsafe margin band";

    EXPECT_FLOAT_EQ(0.0f, model.baked_vertex(target).weight[0]);
}

TEST(BowlModelCameraCtor, LutCamera_BakesVisibleUv) {
    LutParams lut;
    lut.points = {{0.0, 0.0}, {120.0, 4.0}};
    lut.width = 640;
    lut.height = 480;
    lut.cx = 320.0;
    lut.cy = 240.0;

    const auto extr = MakeSideCameras({0})[0];
    std::vector<Camera> cams;
    cams.emplace_back(extr, MakeCameraModel("table", lut));

    const BowlParams params(2.0, 1.0, 0.5, 0.2, 4.0, 0.1, 2.2, 4.0,
                            cv::Vec2d(0.0, 0.0));
    const BowlModel model(params, cams);
    ASSERT_GT(model.vertex_count(), 0);
    ASSERT_EQ(1, model.baked_num_cameras());

    int weighted = 0;
    for (int i = 0; i < model.vertex_count(); ++i) {
        const BowlModel::BakedVertex& baked_vertex = model.baked_vertex(i);
        if (baked_vertex.weight[0] > 0.0f) {
            ++weighted;
            EXPECT_GE(baked_vertex.uv[0][0], 0.0f);
            EXPECT_LE(baked_vertex.uv[0][0], 1.0f);
            EXPECT_GE(baked_vertex.uv[0][1], 0.0f);
            EXPECT_LE(baked_vertex.uv[0][1], 1.0f);
        }
    }
    EXPECT_GT(weighted, 0);
}
