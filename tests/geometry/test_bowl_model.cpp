// Tests for avm_stitch bowl model (BowlParams + BowlModel).
// Pure math, no GL / OSG dependency.

#include <gtest/gtest.h>

#include "geometry/bowl_model.hpp"

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
