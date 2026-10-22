#include <gtest/gtest.h>

#include "bowl_model/bowl_model.hpp"
#include "bowl_model/bowl_params.hpp"

#include "test_support/stitch_test_helpers.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

using namespace avm_stitch;

using avm_stitch::test::DefaultBakeCameras;
using avm_stitch::test::DefaultIntrinsics;
using avm_stitch::test::MakeLookAtCamera;
using avm_stitch::test::MakeRealIntrinsics;
using avm_stitch::test::WrapKbCameras;

TEST(BowlParamsTest, AccessorsRoundTrip) {
    const BowlParams p(4.0, 2.0, 0.5, 0.2, 12.6, 0.09, 2.2, 8.6,
                       cv::Vec2d(1.5, 0.0));
    EXPECT_DOUBLE_EQ(0.5, p.FlatPlatformRadius());
    EXPECT_DOUBLE_EQ(0.2, p.GridStep());
    EXPECT_DOUBLE_EQ(12.6, p.OuterRadius());
    EXPECT_DOUBLE_EQ(0.09, p.SlopeCoefficient());
    EXPECT_DOUBLE_EQ(2.2, p.HeightExponent());
    EXPECT_DOUBLE_EQ(8.6, p.MaxHeight());
    EXPECT_DOUBLE_EQ(1.5, p.CarCenter()[0]);
    EXPECT_DOUBLE_EQ(0.0, p.CarCenter()[1]);
    EXPECT_NEAR(0.4135312, p.BowlHeight(2.0), 1e-6);
    EXPECT_EQ(126, p.GridSegmentsPerAxis());
}

TEST(BowlParamsTest, FlatRegionHalfExtents) {
    const BowlParams p(4.0, 2.0, 0.5, 0.2, 12.6, 0.09, 2.2, 8.6);
    EXPECT_DOUBLE_EQ(0.0, p.DistanceFromFlatPlatform(4.0 / 2.0 + 0.5, 0.0));
    EXPECT_DOUBLE_EQ(0.0, p.DistanceFromFlatPlatform(0.0, 2.0 / 2.0 + 0.5));
    EXPECT_GT(p.DistanceFromFlatPlatform(4.0 / 2.0 + 0.5 + 1.0, 0.0), 0.0);
    EXPECT_GT(p.DistanceFromFlatPlatform(0.0, 2.0 / 2.0 + 0.5 + 1.0), 0.0);
}

TEST(BowlParamsTest, RdMatchesHandComputed) {
    const BowlParams p(4.0, 2.0, 1.0, 0.2, 12.6, 0.09, 2.2, 8.6);
    EXPECT_DOUBLE_EQ(0.0, p.DistanceFromFlatPlatform(3.0, 0.0));
    EXPECT_DOUBLE_EQ(2.0, p.DistanceFromFlatPlatform(5.0, 0.0));
    EXPECT_NEAR(std::sqrt(2.0 * 2.0 + 2.0 * 2.0), p.DistanceFromFlatPlatform(5.0, 4.0), 1e-7);
    const BowlParams shifted(4.0, 2.0, 1.0, 0.2, 12.6, 0.09, 2.2, 8.6,
                             cv::Vec2d(2.0, 0.0));
    EXPECT_DOUBLE_EQ(2.0, shifted.DistanceFromFlatPlatform(7.0, 0.0));
}

TEST(BowlParamsTest, BowlZValues) {
    const BowlParams p(4.0, 2.0, 0.5, 0.2, 12.6, 0.09, 2.2, 8.6);
    EXPECT_DOUBLE_EQ(0.0, p.BowlHeight(0.0));
    EXPECT_NEAR(0.4135312, p.BowlHeight(2.0), 1e-6);
}

TEST(BowlParamsTest, GridSegments) {
    const BowlParams p(4.0, 2.0, 0.5, 0.2, 12.6, 0.09, 2.2, 8.6);
    EXPECT_EQ(126, p.GridSegmentsPerAxis());
}

TEST(BowlParamsTest, FullCtorOverrideKeepsOriginalIntact) {
    const BowlParams original = BowlParams::ForCarSize(5.6, 2.2, 2.9);
    const BowlParams modified(5.6, 2.2, original.FlatPlatformRadius(), 0.0,
                              original.OuterRadius(), original.SlopeCoefficient(),
                              original.HeightExponent(), original.MaxHeight(),
                              original.CarCenter());

    EXPECT_DOUBLE_EQ(0.0, modified.GridStep());
    EXPECT_DOUBLE_EQ(0.2, original.GridStep());
    EXPECT_DOUBLE_EQ(2.0, original.FlatPlatformRadius());
    EXPECT_DOUBLE_EQ(12.6, original.OuterRadius());
    EXPECT_DOUBLE_EQ(0.09, original.SlopeCoefficient());
    EXPECT_DOUBLE_EQ(2.2, original.HeightExponent());
    EXPECT_DOUBLE_EQ(8.6, original.MaxHeight());
    EXPECT_DOUBLE_EQ(1.45, original.CarCenter()[0]);
    EXPECT_DOUBLE_EQ(0.0, original.CarCenter()[1]);
}

TEST(BowlParamsTest, ForCarSizeScaling) {
    const BowlParams p1 = BowlParams::ForCarSize(5.6, 2.2, 2.9);
    const BowlParams p2 = BowlParams::ForCarSize(11.2, 2.2, 2.9);
    EXPECT_NEAR(2.0, p2.OuterRadius() / p1.OuterRadius(), 1e-12);
    EXPECT_NEAR(2.0, p2.GridStep() / p1.GridStep(), 1e-12);
    EXPECT_NEAR(2.0, p2.FlatPlatformRadius() / p1.FlatPlatformRadius(), 1e-12);
    EXPECT_NEAR(2.0, p2.MaxHeight() / p1.MaxHeight(), 1e-12);
    EXPECT_NEAR(std::pow(2.0, -1.2), p2.SlopeCoefficient() / p1.SlopeCoefficient(), 1e-12);
}

namespace {

BowlParams MeshParams() {
    const BowlParams derived = BowlParams::ForCarSize(5.6, 2.2, 2.9);
    return BowlParams(5.6, 2.2, derived.FlatPlatformRadius(), derived.GridStep(),
                      derived.OuterRadius(), derived.SlopeCoefficient(),
                      derived.HeightExponent(), derived.MaxHeight(), cv::Vec2d(0.0, 0.0));
}

}

TEST(BowlParamsTest, GenerateBlockBoatMesh_MeshStructure) {
    const BowlParams p = MeshParams();
    std::vector<cv::Vec3d> vertices, normals;
    std::vector<std::array<int, 3>> triangles;
    p.GenerateBlockBoatMesh(vertices, normals, triangles);

    ASSERT_GT(vertices.size(), 0u);
    ASSERT_EQ(vertices.size(), normals.size());
    ASSERT_GT(triangles.size(), 0u);
    ASSERT_EQ(0, triangles.size() % 2);
    ASSERT_GE(triangles.size(), 2u);

    for (size_t i = 0; i < vertices.size(); ++i) {
        EXPECT_GE(vertices[i][2], 0.0);
        EXPECT_LE(vertices[i][2], p.MaxHeight() + 1e-9);
        EXPECT_NEAR(1.0, cv::norm(normals[i]), 1e-9);
    }
    for (const auto& tri : triangles) {
        for (int idx : tri) {
            EXPECT_GE(idx, 0);
            EXPECT_LT(idx, static_cast<int>(vertices.size()));
        }
    }
}

TEST(BowlParamsTest, GenerateBlockBoatMesh_CircleClip) {
    const BowlParams p = MeshParams();
    std::vector<cv::Vec3d> vertices, normals;
    std::vector<std::array<int, 3>> triangles;
    p.GenerateBlockBoatMesh(vertices, normals, triangles);
    ASSERT_GT(vertices.size(), 0u);
    for (const cv::Vec3d& v : vertices) {
        EXPECT_LE(p.DistanceFromFlatPlatform(v[0], v[1]), p.OuterRadius() + 1e-9);
    }
}

TEST(BowlParamsTest, GenerateBlockBoatMesh_MatchesBowlModelConstructorPath) {
    const BowlParams p = MeshParams();
    std::vector<cv::Vec3d> vertices, normals;
    std::vector<std::array<int, 3>> triangles;
    p.GenerateBlockBoatMesh(vertices, normals, triangles);

    const BowlModel model(p, test::WrapKbSideCameras({0, 180, 90, -90}));

    ASSERT_EQ(static_cast<int>(vertices.size()), model.vertex_count());
    ASSERT_EQ(static_cast<int>(normals.size()), model.normal_count());
    ASSERT_EQ(triangles.size(),
              static_cast<size_t>(model.triangle_count()));

    for (int i = 0; i < model.vertex_count(); ++i) {
        const cv::Vec3d& dv = vertices[static_cast<size_t>(i)];
        const cv::Vec3d& mv = model.vertex(i);
        EXPECT_EQ(dv[0], mv[0]);
        EXPECT_EQ(dv[1], mv[1]);
        EXPECT_EQ(dv[2], mv[2]);
        const cv::Vec3d& dn = normals[static_cast<size_t>(i)];
        const cv::Vec3d& mn = model.normal(i);
        EXPECT_EQ(dn[0], mn[0]);
        EXPECT_EQ(dn[1], mn[1]);
        EXPECT_EQ(dn[2], mn[2]);
    }
    for (int t = 0; t < model.triangle_count(); ++t) {
        EXPECT_EQ(triangles[static_cast<size_t>(t)][0], model.triangle(t)[0]);
        EXPECT_EQ(triangles[static_cast<size_t>(t)][1], model.triangle(t)[1]);
        EXPECT_EQ(triangles[static_cast<size_t>(t)][2], model.triangle(t)[2]);
    }
}

TEST(ForCarSizeTest, CarScale_MatchesCalibratedValues) {
    const BowlParams p = BowlParams::ForCarSize(5.6, 2.2, 2.9);
    EXPECT_NEAR(12.6, p.OuterRadius(), 12.6 * 0.20);
    EXPECT_NEAR(2.0, p.FlatPlatformRadius(), 2.0 * 0.20);
    EXPECT_NEAR(0.2, p.GridStep(), 0.2 * 0.20);
    EXPECT_NEAR(0.09, p.SlopeCoefficient(), 0.09 * 0.20);
    EXPECT_DOUBLE_EQ(2.2, p.HeightExponent());
    EXPECT_NEAR(8.6, p.MaxHeight(), 8.6 * 0.20);
    EXPECT_DOUBLE_EQ(1.45, p.CarCenter()[0]);
    EXPECT_DOUBLE_EQ(0.0, p.CarCenter()[1]);
}

TEST(ForCarSizeTest, CarAnchor_ExactRegressionToOldDefault) {
    const BowlParams p = BowlParams::ForCarSize(5.6, 2.2, 2.9);
    EXPECT_DOUBLE_EQ(12.6, p.OuterRadius());
    EXPECT_DOUBLE_EQ(2.0, p.FlatPlatformRadius());
    EXPECT_DOUBLE_EQ(0.2, p.GridStep());
    EXPECT_DOUBLE_EQ(8.6, p.MaxHeight());
    EXPECT_DOUBLE_EQ(0.09, p.SlopeCoefficient());
    EXPECT_DOUBLE_EQ(2.2, p.HeightExponent());
    const double rd_star = std::pow(p.MaxHeight() / p.SlopeCoefficient(), 1.0 / p.HeightExponent());
    EXPECT_NEAR(7.9454, rd_star, 1e-4);
    EXPECT_NEAR(0.6306, rd_star / p.OuterRadius(), 1e-4);
    EXPECT_LT(rd_star, p.OuterRadius());
    EXPECT_DOUBLE_EQ(1.45, p.CarCenter()[0]);
    EXPECT_DOUBLE_EQ(0.0, p.CarCenter()[1]);
}

TEST(ForCarSizeTest, LinearSimilarity_DoublingLengthDoublesAllLengths) {
    const BowlParams p1 = BowlParams::ForCarSize(5.6, 2.2, 2.9);
    const BowlParams p2 = BowlParams::ForCarSize(11.2, 2.2, 2.9);
    ASSERT_GT(p1.SlopeCoefficient(), 0.0);
    ASSERT_GT(p1.MaxHeight(), 0.0);
    ASSERT_GT(p1.GridStep(), 0.0);
    ASSERT_GT(p1.FlatPlatformRadius(), 0.0);
    ASSERT_GT(p1.OuterRadius(), 0.0);
    EXPECT_NEAR(2.0, p2.OuterRadius() / p1.OuterRadius(), 1e-9);
    EXPECT_NEAR(2.0, p2.FlatPlatformRadius() / p1.FlatPlatformRadius(), 1e-9);
    EXPECT_NEAR(2.0, p2.GridStep() / p1.GridStep(), 1e-9);
    EXPECT_NEAR(2.0, p2.MaxHeight() / p1.MaxHeight(), 1e-9);
    EXPECT_DOUBLE_EQ(p1.HeightExponent(), p2.HeightExponent());
    EXPECT_DOUBLE_EQ(2.2, p1.HeightExponent());
    EXPECT_NEAR(std::pow(2.0, -1.2), p2.SlopeCoefficient() / p1.SlopeCoefficient(), 1e-9);
}

TEST(ForCarSizeTest, SelfSimilarity_OpeningRatioConstantAcrossScales) {
    const std::vector<double> scales = {5.6, 11.2, 44.8, 211.583};
    double ref_ratio = 0.0;
    for (double L : scales) {
        const BowlParams p = BowlParams::ForCarSize(L, 2.2, L / 2.0);
        const double rd_star = std::pow(p.MaxHeight() / p.SlopeCoefficient(), 1.0 / p.HeightExponent());
        const double opening = rd_star / p.OuterRadius();
        if (ref_ratio == 0.0) {
            ref_ratio = opening;
            EXPECT_NEAR(0.63, ref_ratio, 0.63 * 0.01) << "anchor opening";
        } else {
            EXPECT_NEAR(ref_ratio, opening, ref_ratio * 1e-9) << "L=" << L;
        }
        EXPECT_LT(rd_star, p.OuterRadius()) << "L=" << L;
    }
}

TEST(ForCarSizeTest, IsotropicRimHeight_LongitudinalVsLateral) {
    struct Car {
        double length, width, axis_x;
    };
    const Car kCars[] = {
        {5.6, 2.2, 2.9},
        {5.27, 2.17, 2.9},
        {211.583, 53.453, 116.371},
    };
    for (const auto& c : kCars) {
        const BowlParams p = BowlParams::ForCarSize(c.length, c.width, c.axis_x);
        auto rim_z = [&p](double rd) {
            const double z = p.SlopeCoefficient() * std::pow(rd, p.HeightExponent());
            return std::min(z, p.MaxHeight());
        };
        const double rd_long = p.OuterRadius() - (c.length / 2.0 + p.FlatPlatformRadius());
        const double rd_lat = p.OuterRadius() - (c.width / 2.0 + p.FlatPlatformRadius());
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

TEST(ForCarSizeTest, ShipScale_CoversCameras) {
    const BowlParams p = BowlParams::ForCarSize(211.583, 53.453, 116.371);
    EXPECT_GE(p.OuterRadius(), 150.0);
    const double rd_star = std::pow(p.MaxHeight() / p.SlopeCoefficient(), 1.0 / p.HeightExponent());
    EXPECT_GE(rd_star, 155.0);
    EXPECT_LE(rd_star, p.OuterRadius());
    EXPECT_DOUBLE_EQ(58.1855, p.CarCenter()[0]);
    EXPECT_DOUBLE_EQ(0.0, p.CarCenter()[1]);
    EXPECT_GT(p.GridStep(), 0.0);
    EXPECT_GT(p.SlopeCoefficient(), 0.0);
    EXPECT_GT(p.MaxHeight(), 0.0);
    EXPECT_DOUBLE_EQ(2.2, p.HeightExponent());
}

TEST(ForCarSizeTest, ShipScale_All8CamerasSeeBowlWithWeight) {
    const BowlParams p = BowlParams::ForCarSize(211.583, 53.453, 116.371);
    ASSERT_GE(p.OuterRadius(), 150.0);
    const double kCamXY[8][2] = {
        {96.71248, 0.0},
        {75.55415, 25.65737},
        {17.36875, 25.65737},
        {-40.81666, 25.65737},
        {75.55415, -25.65737},
        {17.36875, -25.65737},
        {-40.81666, -25.65737},
        {-91.59664, 0.0},
    };
    const double kCamZ[8] = {19.00911, 30.88980, 30.88980, 30.88980,
                             30.88980, 30.88980, 30.88980, 28.51366};
    std::vector<KbParams> intrs(8, MakeRealIntrinsics());
    std::vector<CameraExtrinsics> extrs;
    for (int j = 0; j < 8; ++j) {
        const double r = std::hypot(kCamXY[j][0], kCamXY[j][1]);
        const double az = std::atan2(kCamXY[j][1], kCamXY[j][0]) * 180.0 / M_PI;
        extrs.push_back(test::MakeLookAtCamera(az, r, kCamZ[j], 0.0));
    }
    const BowlModel model(p, WrapKbCameras(intrs, extrs));
    ASSERT_GT(model.vertex_count(), 0);
    for (int j = 0; j < 8; ++j) {
        int weighted = 0;
        for (int i = 0; i < model.vertex_count(); ++i) {
            if (model.baked_vertex(i).weight[j] > 1e-9)
                ++weighted;
        }
        EXPECT_GT(weighted, 0) << "camera " << j << " has no weighted vertices";
    }
}

TEST(ForCarSizeTest, GridTopology_Constant126x126AcrossScales) {
    for (double L : {5.6, 18.0, 211.583, 423.166}) {
        const BowlParams p = BowlParams::ForCarSize(L, 2.2, L / 2.0);
        const double ratio = p.OuterRadius() / p.GridStep();
        EXPECT_NEAR(63.0, ratio, 1.0) << "L=" << L;
        const long rows = static_cast<long>(std::lround(ratio)) * 2;
        EXPECT_EQ(126, rows) << "L=" << L;
        const long max_vertices = (rows + 1) * (rows + 1);
        EXPECT_LE(max_vertices, 150000) << "L=" << L;
        EXPECT_GT(max_vertices, 0) << "L=" << L;
    }
}

TEST(BowlParamsTest, DefaultCarCenter_BackwardCompatVertexCount) {
    const BowlParams p;
    auto model = BowlModel(p, WrapKbCameras(DefaultIntrinsics(), DefaultBakeCameras()));
    int count_default = model.vertex_count();

    const BowlParams p_centered(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                cv::Vec2d(0.0, 0.0));
    auto model2 = BowlModel(p_centered, WrapKbCameras(DefaultIntrinsics(), DefaultBakeCameras()));
    EXPECT_EQ(count_default, model2.vertex_count());
}

TEST(BowlParamsTest, ComputeRd_CarCenterShiftsFlatRegion) {
    const BowlParams params(2.0, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
                            cv::Vec2d(2.0, 0.0));

    EXPECT_DOUBLE_EQ(0.0, params.DistanceFromFlatPlatform(3.0, 0.0));

    const BowlParams default_params(2.0, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0);
    EXPECT_GT(default_params.DistanceFromFlatPlatform(3.0, 0.0), 0.0);
}
