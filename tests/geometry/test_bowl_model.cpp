// Tests for avm_stitch bowl model (BowlParams + BowlMesh).
// Pure math, no GL / OSG dependency.

#include <gtest/gtest.h>

#include "geometry/bowl_model.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace avm_stitch;

namespace {

// Count vertices that lie strictly inside the flat (rd == 0) region.
size_t CountFlatVertices(const BowlMesh& mesh, double x_half, double y_half) {
    size_t count = 0;
    for (int i = 0; i < mesh.vertex_count(); ++i) {
        const cv::Vec3d& v = mesh.vertex(i);
        if (std::abs(v[0]) < x_half - 1e-9 && std::abs(v[1]) < y_half - 1e-9) {
            count++;
        }
    }
    return count;
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
// BowlMesh construction
// ---------------------------------------------------------------------------

TEST(BowlMeshTest, ReturnsNonEmptyMesh) {
    const BowlMesh mesh(BowlParams::Default());
    EXPECT_GT(mesh.vertex_count(), 0);
    EXPECT_GT(mesh.triangle_count(), 0);
    EXPECT_GT(mesh.rows(), 0);
    EXPECT_GT(mesh.cols(), 0);
}

TEST(BowlMeshTest, GridSizeMatchesParams) {
    const BowlMesh mesh(BowlParams::Default());
    // r_outter=12.6, step=0.1 -> (12.6/0.1)*2 = 252 (rounded, not truncated)
    EXPECT_EQ(252, mesh.rows());
    EXPECT_EQ(252, mesh.cols());
}

TEST(BowlMeshTest, VerticesInXyBounds) {
    const BowlMesh mesh(BowlParams::Default());
    ASSERT_GT(mesh.vertex_count(), 0);
    for (int i = 0; i < mesh.vertex_count(); ++i) {
        const cv::Vec3d& v = mesh.vertex(i);
        EXPECT_GE(v[0], -12.6 - 1e-9);
        EXPECT_LE(v[0], 12.6 + 1e-9);
        EXPECT_GE(v[1], -12.6 - 1e-9);
        EXPECT_LE(v[1], 12.6 + 1e-9);
    }
}

TEST(BowlMeshTest, VerticesZInRange) {
    const BowlMesh mesh(BowlParams::Default());
    ASSERT_GT(mesh.vertex_count(), 0);
    for (int i = 0; i < mesh.vertex_count(); ++i) {
        const cv::Vec3d& v = mesh.vertex(i);
        EXPECT_GE(v[2], 0.0);
        EXPECT_LE(v[2], 8.6 + 1e-9);
    }
}

TEST(BowlMeshTest, FlatCenterRegion) {
    BowlParams params;
    params.car_length = 2.0;
    params.car_width = 1.0;
    params.round_l = 0.5;
    double x_half = params.car_length / 2.0 + params.round_l; // 1.5
    double y_half = params.car_width / 2.0 + params.round_l;  // 1.0
    const BowlMesh mesh(params);
    bool found_flat = false;
    for (int i = 0; i < mesh.vertex_count(); ++i) {
        const cv::Vec3d& v = mesh.vertex(i);
        if (std::abs(v[0]) < x_half - 1e-9 && std::abs(v[1]) < y_half - 1e-9) {
            found_flat = true;
            EXPECT_DOUBLE_EQ(0.0, v[2]);
        }
    }
    EXPECT_TRUE(found_flat);
}

TEST(BowlMeshTest, EdgesMonotonic) {
    // Along a fixed row (fixed u/x), z must be a non-decreasing function of rd.
    const BowlMesh mesh(BowlParams::Default());
    ASSERT_GT(mesh.rows(), 0);
    const double x_half = 5.6 / 2.0 + 2.0;
    const double y_half = 2.2 / 2.0 + 2.0;
    const double r_outter = 12.6;

    // Group vertices into rows (recover grid row i from world x), O(n).
    std::vector<std::vector<std::pair<double, double>>> rows(mesh.rows() + 1);
    for (int k = 0; k < mesh.vertex_count(); ++k) {
        const cv::Vec3d& v = mesh.vertex(k);
        double u = v[0] / r_outter;
        int i = std::lround((u + 1.0) / 2.0 * mesh.rows());
        double dx = std::max(0.0, std::abs(v[0]) - x_half);
        double dy = std::max(0.0, std::abs(v[1]) - y_half);
        double rd = std::sqrt(dx * dx + dy * dy);
        if (i >= 0 && i <= mesh.rows()) rows[i].emplace_back(rd, v[2]);
    }

    for (int i = 0; i <= mesh.rows(); ++i) {
        auto& row = rows[i];
        if (row.size() < 2) continue;
        std::sort(row.begin(), row.end());
        for (size_t k = 1; k < row.size(); ++k) {
            EXPECT_GE(row[k].second, row[k - 1].second - 1e-9)
                << "row " << i << ": z decreases when rd increases";
        }
    }
}

TEST(BowlMeshTest, NormalsAreUnitLength) {
    const BowlMesh mesh(BowlParams::Default());
    ASSERT_EQ(mesh.vertex_count(), mesh.normal_count());
    for (int i = 0; i < mesh.normal_count(); ++i) {
        const cv::Vec3d& n = mesh.normal(i);
        double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        EXPECT_NEAR(1.0, len, 1e-6);
    }
}

TEST(BowlMeshTest, FlatCenterNormalsUpward) {
    BowlParams params;
    params.car_length = 2.0;
    params.car_width = 1.0;
    params.round_l = 0.5;
    double x_half = 1.5;
    double y_half = 1.0;
    const BowlMesh mesh(params);
    bool found_flat = false;
    for (int k = 0; k < mesh.vertex_count(); ++k) {
        const cv::Vec3d& v = mesh.vertex(k);
        if (std::abs(v[0]) < x_half - 1e-9 && std::abs(v[1]) < y_half - 1e-9) {
            found_flat = true;
            const cv::Vec3d& n = mesh.normal(k);
            EXPECT_NEAR(0.0, n[0], 1e-6);
            EXPECT_NEAR(0.0, n[1], 1e-6);
            EXPECT_NEAR(1.0, n[2], 1e-6);
        }
    }
    EXPECT_TRUE(found_flat);
}

TEST(BowlMeshTest, TrianglesHaveValidIndices) {
    const BowlMesh mesh(BowlParams::Default());
    ASSERT_GT(mesh.triangle_count(), 0);
    for (int t = 0; t < mesh.triangle_count(); ++t) {
        for (int idx : mesh.triangle(t)) {
            EXPECT_GE(idx, 0);
            EXPECT_LT(idx, mesh.vertex_count());
        }
    }
}

TEST(BowlMeshTest, DifferentCarSizeChangesFlatRegion) {
    BowlParams truck_params;
    truck_params.car_length = 18.0;
    truck_params.car_width = 2.5;
    truck_params.step = 0.5;   // coarse for speed
    const BowlMesh truck_mesh(truck_params);
    EXPECT_GT(truck_mesh.vertex_count(), 0);

    BowlParams default_params;
    default_params.step = 0.5; // coarse for speed
    const BowlMesh default_mesh(default_params);

    size_t truck_flat = CountFlatVertices(truck_mesh, 18.0 / 2.0 + 2.0, 2.5 / 2.0 + 2.0);
    size_t default_flat = CountFlatVertices(default_mesh, 5.6 / 2.0 + 2.0, 2.2 / 2.0 + 2.0);
    EXPECT_GT(truck_flat, default_flat);
}

TEST(BowlMeshTest, MinimalParams_OneCellMesh) {
    BowlParams params;
    params.step = 12.6;    // step == r_outter -> 1 cell per axis
    params.r_outter = 12.6;
    const BowlMesh mesh(params);
    EXPECT_EQ(2, mesh.rows());
    EXPECT_EQ(2, mesh.cols());
    EXPECT_GT(mesh.vertex_count(), 0);   // center + edge points retained
    // must not crash; indices still valid
    for (int t = 0; t < mesh.triangle_count(); ++t) {
        for (int idx : mesh.triangle(t)) {
            EXPECT_LT(idx, mesh.vertex_count());
        }
    }
}

// ---------------------------------------------------------------------------
// Fine-grained index access: counts + per-element queries (no container
// exposure). These tests define the new read API surface.
// ---------------------------------------------------------------------------

TEST(BowlMeshTest, IndexAccess_CountsAreConsistent) {
    const BowlMesh mesh(BowlParams::Default());
    EXPECT_GT(mesh.vertex_count(), 0);
    EXPECT_GT(mesh.triangle_count(), 0);
    // Construction invariant: one normal per vertex.
    EXPECT_EQ(mesh.vertex_count(), mesh.normal_count());
}

TEST(BowlMeshTest, IndexAccess_VertexInRange) {
    const BowlMesh mesh(BowlParams::Default());
    // Every indexed vertex lies in the same XY/Z bounds as the mesh.
    for (int i = 0; i < mesh.vertex_count(); ++i) {
        const cv::Vec3d& v = mesh.vertex(i);
        EXPECT_GE(v[0], -12.6 - 1e-9);
        EXPECT_LE(v[0], 12.6 + 1e-9);
        EXPECT_GE(v[1], -12.6 - 1e-9);
        EXPECT_LE(v[1], 12.6 + 1e-9);
        EXPECT_GE(v[2], 0.0);
        EXPECT_LE(v[2], 8.6 + 1e-9);
    }
}

TEST(BowlMeshTest, IndexAccess_NormalIsUnit) {
    const BowlMesh mesh(BowlParams::Default());
    for (int i = 0; i < mesh.normal_count(); ++i) {
        const cv::Vec3d& n = mesh.normal(i);
        double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        EXPECT_NEAR(1.0, len, 1e-6);
    }
}

TEST(BowlMeshTest, IndexAccess_TriangleIndicesValid) {
    const BowlMesh mesh(BowlParams::Default());
    for (int t = 0; t < mesh.triangle_count(); ++t) {
        const std::array<int, 3> tri = mesh.triangle(t);
        for (int idx : tri) {
            EXPECT_GE(idx, 0);
            EXPECT_LT(idx, mesh.vertex_count());
        }
    }
}

TEST(BowlMeshTest, IndexAccess_WrappingConstructorCounts) {
    // Wrap-constructor mesh: counts reflect the wrapped data, triangles empty.
    BowlMesh mesh({cv::Vec3d(0.0, 0.0, 10.0)}, {cv::Vec3d(0.0, 0.0, 1.0)});
    EXPECT_EQ(1, mesh.vertex_count());
    EXPECT_EQ(1, mesh.normal_count());
    EXPECT_EQ(0, mesh.triangle_count());
    EXPECT_DOUBLE_EQ(10.0, mesh.vertex(0)[2]);
    EXPECT_NEAR(1.0, mesh.normal(0)[2], 1e-9);
}
