//

#include <gtest/gtest.h>

#include "bowl_model/bowl_params.hpp"
#include "bowl_model/bowl_model.hpp"

#include "test_support/geometry_test_helpers.hpp"

#include <cmath>
#include <vector>

#include <array>

using namespace avm_stitch;

TEST(BowlParamsTest, AccessorsRoundTrip) {
    const BowlParams p(4.0, 2.0, 0.5, 0.2, 12.6, 0.09, 2.2, 8.6,
                       cv::Vec2d(1.5, 0.0));
        EXPECT_DOUBLE_EQ(0.5, p.round_l());
    EXPECT_DOUBLE_EQ(0.2, p.step());
    EXPECT_DOUBLE_EQ(12.6, p.r_outter());
    EXPECT_DOUBLE_EQ(0.09, p.k_slop());
    EXPECT_DOUBLE_EQ(2.2, p.z_exp());
    EXPECT_DOUBLE_EQ(8.6, p.max_z());
    EXPECT_DOUBLE_EQ(1.5, p.car_center()[0]);
    EXPECT_DOUBLE_EQ(0.0, p.car_center()[1]);
        EXPECT_DOUBLE_EQ(2.5, p.x_half());       EXPECT_DOUBLE_EQ(1.5, p.y_half());       EXPECT_NEAR(0.4135312, p.bowl_z(2.0), 1e-6);       EXPECT_EQ(126, p.grid_segments());              }

TEST(BowlParamsTest, XHalfYHalf) {
    const BowlParams p(4.0, 2.0, 0.5, 0.2, 12.6, 0.09, 2.2, 8.6);
    EXPECT_DOUBLE_EQ(2.5, p.x_half());       EXPECT_DOUBLE_EQ(1.5, p.y_half());   }

TEST(BowlParamsTest, RdMatchesHandComputed) {
    const BowlParams p(4.0, 2.0, 1.0, 0.2, 12.6, 0.09, 2.2, 8.6);       EXPECT_DOUBLE_EQ(0.0, p.rd(3.0, 0.0));           EXPECT_DOUBLE_EQ(2.0, p.rd(5.0, 0.0));           EXPECT_NEAR(std::sqrt(2.0 * 2.0 + 2.0 * 2.0), p.rd(5.0, 4.0), 1e-7);
                    const BowlParams shifted(4.0, 2.0, 1.0, 0.2, 12.6, 0.09, 2.2, 8.6,
                             cv::Vec2d(2.0, 0.0));
    EXPECT_DOUBLE_EQ(2.0, shifted.rd(7.0, 0.0));
}

TEST(BowlParamsTest, BowlZValues) {
    const BowlParams p(4.0, 2.0, 0.5, 0.2, 12.6, 0.09, 2.2, 8.6);
    EXPECT_DOUBLE_EQ(0.0, p.bowl_z(0.0));       EXPECT_NEAR(0.4135312, p.bowl_z(2.0), 1e-6);
}

TEST(BowlParamsTest, GridSegments) {
    const BowlParams p(4.0, 2.0, 0.5, 0.2, 12.6, 0.09, 2.2, 8.6);
    EXPECT_EQ(126, p.grid_segments());
}

TEST(BowlParamsTest, WithReturnsCopyOriginalIntact) {
    const BowlParams original = DeriveBowlParams(5.6, 2.2, 2.9);
    const BowlParams modified = original.with_step(0.0);

    EXPECT_DOUBLE_EQ(0.0, modified.step());
                EXPECT_DOUBLE_EQ(0.2, original.step());
    EXPECT_DOUBLE_EQ(2.0, original.round_l());
    EXPECT_DOUBLE_EQ(12.6, original.r_outter());
    EXPECT_DOUBLE_EQ(0.09, original.k_slop());
    EXPECT_DOUBLE_EQ(2.2, original.z_exp());
    EXPECT_DOUBLE_EQ(8.6, original.max_z());
    EXPECT_DOUBLE_EQ(1.45, original.car_center()[0]);       EXPECT_DOUBLE_EQ(0.0, original.car_center()[1]);
}

TEST(BowlParamsTest, DeriveBowlParamsScaling) {
    const BowlParams p1 = DeriveBowlParams(5.6, 2.2, 2.9);        const BowlParams p2 = DeriveBowlParams(11.2, 2.2, 2.9);       EXPECT_NEAR(2.0, p2.r_outter() / p1.r_outter(), 1e-12);
    EXPECT_NEAR(2.0, p2.step() / p1.step(), 1e-12);
    EXPECT_NEAR(2.0, p2.round_l() / p1.round_l(), 1e-12);
    EXPECT_NEAR(2.0, p2.max_z() / p1.max_z(), 1e-12);
    EXPECT_NEAR(std::pow(2.0, -1.2), p2.k_slop() / p1.k_slop(), 1e-12);
}


namespace {

BowlParams MeshParams() {
    return DeriveBowlParams(5.6, 2.2, 2.9).with_car_center(cv::Vec2d(0.0, 0.0));
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
        EXPECT_LE(vertices[i][2], p.max_z() + 1e-9);
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
        EXPECT_LE(p.rd(v[0], v[1]), p.r_outter() + 1e-9);
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
