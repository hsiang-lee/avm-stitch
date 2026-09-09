// Tests for avm_stitch bowl model (BowlParams + CreateBowlMesh).
// Pure math, no GL / OSG dependency.

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "geometry/bowl_model.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace avm_stitch;

namespace {

// Count vertices that lie strictly inside the flat (rd == 0) region.
size_t CountFlatVertices(const BowlMesh& mesh, double x_half, double y_half) {
    size_t count = 0;
    for (const auto& v : mesh.vertices) {
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

TEST(BowlParamsTest, FromJson_PriorityCalibrationBowl) {
    nlohmann::json task = {
        {"calibration", {{"bowl", {{"car_length", 18.0}, {"car_width", 2.5}}}}},
        {"bowl", {{"car_length", 3.0}, {"car_width", 1.0}}}
    };
    auto p = BowlParams::FromJson(task);
    // calibration.bowl takes priority over top-level bowl
    EXPECT_DOUBLE_EQ(18.0, p.car_length);
    EXPECT_DOUBLE_EQ(2.5, p.car_width);
}

TEST(BowlParamsTest, FromJson_FallbackToTopLevelBowl) {
    nlohmann::json task = {
        {"bowl", {{"car_length", 18.0}, {"car_width", 2.5}}}
    };
    auto p = BowlParams::FromJson(task);
    EXPECT_DOUBLE_EQ(18.0, p.car_length);
    EXPECT_DOUBLE_EQ(2.5, p.car_width);
}

TEST(BowlParamsTest, FromJson_MissingFieldsUseDefaults) {
    nlohmann::json task = {
        {"calibration", {{"bowl", {{"car_length", 18.0}}}}}
    };
    auto p = BowlParams::FromJson(task);
    EXPECT_DOUBLE_EQ(18.0, p.car_length);
    EXPECT_DOUBLE_EQ(2.2, p.car_width);   // default
    EXPECT_DOUBLE_EQ(2.0, p.round_l);     // default
    EXPECT_DOUBLE_EQ(0.1, p.step);        // default
    EXPECT_DOUBLE_EQ(12.6, p.r_outter);   // default
    EXPECT_DOUBLE_EQ(0.09, p.k_slop);     // default
    EXPECT_DOUBLE_EQ(2.2, p.z_exp);       // default
    EXPECT_DOUBLE_EQ(8.6, p.max_z);       // default
}

TEST(BowlParamsTest, FromJson_EmptyJsonUsesAllDefaults) {
    auto p = BowlParams::FromJson(nlohmann::json::object());
    auto def = BowlParams::Default();
    EXPECT_DOUBLE_EQ(def.car_length, p.car_length);
    EXPECT_DOUBLE_EQ(def.car_width, p.car_width);
    EXPECT_DOUBLE_EQ(def.round_l, p.round_l);
    EXPECT_DOUBLE_EQ(def.step, p.step);
    EXPECT_DOUBLE_EQ(def.r_outter, p.r_outter);
    EXPECT_DOUBLE_EQ(def.k_slop, p.k_slop);
    EXPECT_DOUBLE_EQ(def.z_exp, p.z_exp);
    EXPECT_DOUBLE_EQ(def.max_z, p.max_z);
}

TEST(BowlParamsTest, FromJson_IntegerValuesParsedAsDouble) {
    nlohmann::json task = {
        {"bowl", {{"car_length", 18}, {"car_width", 2}}}   // ints, not doubles
    };
    auto p = BowlParams::FromJson(task);
    EXPECT_DOUBLE_EQ(18.0, p.car_length);
    EXPECT_DOUBLE_EQ(2.0, p.car_width);
}

TEST(BowlParamsTest, FromJson_StringNumberFallsBackToDefault) {
    // String-encoded numbers must not throw; spec says malformed -> default.
    nlohmann::json task = {
        {"bowl", {{"car_length", "18.0"}, {"car_width", "2.5"}}}
    };
    auto p = BowlParams::FromJson(task);
    EXPECT_DOUBLE_EQ(5.6, p.car_length);   // default, string is not numeric
    EXPECT_DOUBLE_EQ(2.2, p.car_width);    // default
}

TEST(BowlParamsTest, FromJson_NullAndWrongTypeFieldsUseDefaults) {
    // null / boolean / object / array values must not throw either.
    nlohmann::json task = {
        {"bowl", {
            {"step", nullptr},
            {"r_outter", true},
            {"k_slop", {{"nested", 1.0}}},
            {"z_exp", {"1.0", "2.0"}}
        }}
    };
    auto p = BowlParams::FromJson(task);
    EXPECT_DOUBLE_EQ(0.1, p.step);        // default
    EXPECT_DOUBLE_EQ(12.6, p.r_outter);   // default
    EXPECT_DOUBLE_EQ(0.09, p.k_slop);     // default
    EXPECT_DOUBLE_EQ(2.2, p.z_exp);       // default
}

// ---------------------------------------------------------------------------
// CreateBowlMesh
// ---------------------------------------------------------------------------

TEST(BowlMeshTest, ReturnsNonEmptyMesh) {
    auto mesh = CreateBowlMesh(BowlParams::Default());
    EXPECT_GT(mesh.vertices.size(), 0u);
    EXPECT_GT(mesh.triangles.size(), 0u);
    EXPECT_GT(mesh.rows, 0);
    EXPECT_GT(mesh.cols, 0);
}

TEST(BowlMeshTest, GridSizeMatchesParams) {
    auto mesh = CreateBowlMesh(BowlParams::Default());
    // r_outter=12.6, step=0.1 -> (12.6/0.1)*2 = 252 (rounded, not truncated)
    EXPECT_EQ(252, mesh.rows);
    EXPECT_EQ(252, mesh.cols);
}

TEST(BowlMeshTest, VerticesInXyBounds) {
    auto mesh = CreateBowlMesh(BowlParams::Default());
    ASSERT_GT(mesh.vertices.size(), 0u);
    for (const auto& v : mesh.vertices) {
        EXPECT_GE(v[0], -12.6 - 1e-9);
        EXPECT_LE(v[0], 12.6 + 1e-9);
        EXPECT_GE(v[1], -12.6 - 1e-9);
        EXPECT_LE(v[1], 12.6 + 1e-9);
    }
}

TEST(BowlMeshTest, VerticesZInRange) {
    auto mesh = CreateBowlMesh(BowlParams::Default());
    ASSERT_GT(mesh.vertices.size(), 0u);
    for (const auto& v : mesh.vertices) {
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
    auto mesh = CreateBowlMesh(params);
    bool found_flat = false;
    for (const auto& v : mesh.vertices) {
        if (std::abs(v[0]) < x_half - 1e-9 && std::abs(v[1]) < y_half - 1e-9) {
            found_flat = true;
            EXPECT_DOUBLE_EQ(0.0, v[2]);
        }
    }
    EXPECT_TRUE(found_flat);
}

TEST(BowlMeshTest, EdgesMonotonic) {
    // Along a fixed row (fixed u/x), z must be a non-decreasing function of rd.
    auto mesh = CreateBowlMesh(BowlParams::Default());
    ASSERT_GT(mesh.rows, 0);
    const double x_half = 5.6 / 2.0 + 2.0;
    const double y_half = 2.2 / 2.0 + 2.0;
    const double r_outter = 12.6;

    // Group vertices into rows (recover grid row i from world x), O(n).
    std::vector<std::vector<std::pair<double, double>>> rows(mesh.rows + 1);
    for (const auto& v : mesh.vertices) {
        double u = v[0] / r_outter;
        int i = std::lround((u + 1.0) / 2.0 * mesh.rows);
        double dx = std::max(0.0, std::abs(v[0]) - x_half);
        double dy = std::max(0.0, std::abs(v[1]) - y_half);
        double rd = std::sqrt(dx * dx + dy * dy);
        if (i >= 0 && i <= mesh.rows) rows[i].emplace_back(rd, v[2]);
    }

    for (int i = 0; i <= mesh.rows; ++i) {
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
    auto mesh = CreateBowlMesh(BowlParams::Default());
    ASSERT_EQ(mesh.vertices.size(), mesh.normals.size());
    for (const auto& n : mesh.normals) {
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
    auto mesh = CreateBowlMesh(params);
    bool found_flat = false;
    for (size_t k = 0; k < mesh.vertices.size(); ++k) {
        const auto& v = mesh.vertices[k];
        if (std::abs(v[0]) < x_half - 1e-9 && std::abs(v[1]) < y_half - 1e-9) {
            found_flat = true;
            const auto& n = mesh.normals[k];
            EXPECT_NEAR(0.0, n[0], 1e-6);
            EXPECT_NEAR(0.0, n[1], 1e-6);
            EXPECT_NEAR(1.0, n[2], 1e-6);
        }
    }
    EXPECT_TRUE(found_flat);
}

TEST(BowlMeshTest, TrianglesHaveValidIndices) {
    auto mesh = CreateBowlMesh(BowlParams::Default());
    ASSERT_GT(mesh.triangles.size(), 0u);
    for (const auto& tri : mesh.triangles) {
        for (int idx : tri) {
            EXPECT_GE(idx, 0);
            EXPECT_LT(idx, static_cast<int>(mesh.vertices.size()));
        }
    }
}

TEST(BowlMeshTest, DifferentCarSizeChangesFlatRegion) {
    BowlParams truck_params;
    truck_params.car_length = 18.0;
    truck_params.car_width = 2.5;
    truck_params.step = 0.5;   // coarse for speed
    auto truck_mesh = CreateBowlMesh(truck_params);
    EXPECT_GT(truck_mesh.vertices.size(), 0u);

    BowlParams default_params;
    default_params.step = 0.5; // coarse for speed
    auto default_mesh = CreateBowlMesh(default_params);

    size_t truck_flat = CountFlatVertices(truck_mesh, 18.0 / 2.0 + 2.0, 2.5 / 2.0 + 2.0);
    size_t default_flat = CountFlatVertices(default_mesh, 5.6 / 2.0 + 2.0, 2.2 / 2.0 + 2.0);
    EXPECT_GT(truck_flat, default_flat);
}

TEST(BowlMeshTest, MinimalParams_OneCellMesh) {
    BowlParams params;
    params.step = 12.6;    // step == r_outter -> 1 cell per axis
    params.r_outter = 12.6;
    auto mesh = CreateBowlMesh(params);
    EXPECT_EQ(2, mesh.rows);
    EXPECT_EQ(2, mesh.cols);
    EXPECT_GT(mesh.vertices.size(), 0u);   // center + edge points retained
    // must not crash; indices still valid
    for (const auto& tri : mesh.triangles) {
        for (int idx : tri) {
            EXPECT_LT(idx, static_cast<int>(mesh.vertices.size()));
        }
    }
}
