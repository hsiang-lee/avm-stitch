#pragma once

// Bowl model: car-parameterized bowl mesh generation + per-vertex bake
// (UV / blend weights) — one geometry class owns both (pure math).
// Algorithm mirrors the reference `create_block_boat_model` (bowlModel.hpp):
//   z = k_slop * rd^z_exp outside the rounded-rectangle flat region,
//   finite-difference normals on the curved rim.
// Mesh generation + bake live in bowl_model.cpp; only lightweight accessors
// are inline here.

#include "geometry/projection.hpp"

#include <opencv2/core.hpp>

#include <array>
#include <cassert>
#include <vector>

namespace avm_stitch {

struct BowlParams {
    double car_length = 5.6;
    double car_width = 2.2;
    double round_l = 2.0;
    double step = 0.1;
    double r_outter = 12.6;
    double k_slop = 0.09;
    double z_exp = 2.2;
    double max_z = 8.6;

    static BowlParams Default() { return BowlParams{}; }
};

// Fixed 8 camera slots (N <= 8). Unused slots stay zero.
struct BakedVertex {
    float uv[8][2];     // normalized [0,1], unused slots = 0
    float weight[8];    // normalized over visible cameras, unused slots = 0

    BakedVertex() {
        for (int j = 0; j < 8; ++j) {
            uv[j][0] = uv[j][1] = 0.0f;
            weight[j] = 0.0f;
        }
    }
};

// Bowl surface triangle mesh generated from car parameters, plus the
// per-vertex baked UV + blend weights after Bake(). Immutable data holder:
// read via the fine-grained index accessors and semantic queries (the
// internal container layout is an implementation detail).
class BowlModel {
public:
    /// Generate the bowl model (VCS coordinates) from car parameters.
    explicit BowlModel(const BowlParams& params);

    /// Wrap pre-built vertex data (for tests / custom geometry): normals are
    /// taken as-is, no grid/normal recomputation, no triangles.
    BowlModel(std::vector<cv::Vec3d> vertices, std::vector<cv::Vec3d> normals);

    // --- fine-grained read access (index + count, no container exposure) ---
    /// Number of vertices (always == normal_count(), a construction invariant).
    int vertex_count() const { return static_cast<int>(vertices_.size()); }
    /// Vertex i in VCS coordinates. Precondition: 0 <= i < vertex_count().
    const cv::Vec3d& vertex(int i) const {
        assert(i >= 0 && i < vertex_count());
        return vertices_[static_cast<size_t>(i)];
    }
    /// Number of normals (always == vertex_count(), a construction invariant).
    int normal_count() const { return static_cast<int>(normals_.size()); }
    /// Unit normal i. Precondition: 0 <= i < normal_count().
    const cv::Vec3d& normal(int i) const {
        assert(i >= 0 && i < normal_count());
        return normals_[static_cast<size_t>(i)];
    }
    /// Number of triangles.
    int triangle_count() const { return static_cast<int>(triangles_.size()); }
    /// Triangle i as vertex indices (by value: small object).
    /// Precondition: 0 <= i < triangle_count().
    std::array<int, 3> triangle(int i) const {
        assert(i >= 0 && i < triangle_count());
        return triangles_[static_cast<size_t>(i)];
    }

    int rows() const { return rows_; }
    int cols() const { return cols_; }

    // --- bake (per-vertex UV + blend weights) ---
    /// Bake per-vertex UV coordinates and blend weights for N cameras
    /// (1 <= N <= 8). Re-baking overwrites previous results. A camera slot
    /// the vertex cannot see keeps zero weight.
    /// @param intrinsics  camera intrinsics (size N)
    /// @param extrinsics  camera extrinsics T_wc (size N, must match intrinsics)
    void Bake(const std::vector<CameraIntrinsics>& intrinsics,
              const std::vector<CameraExtrinsics>& extrinsics);

    /// True if Bake() has been called on this model.
    bool IsBaked() const { return baked_num_cameras_ > 0; }
    /// Number of cameras from the last Bake() call.
    /// Precondition: IsBaked().
    int baked_num_cameras() const {
        assert(baked_num_cameras_ > 0);
        return baked_num_cameras_;
    }
    /// Baked vertex i (one per bowl vertex).
    /// Precondition: IsBaked() && 0 <= i < vertex_count().
    const BakedVertex& baked_vertex(int i) const {
        assert(IsBaked());
        assert(i >= 0 && i < static_cast<int>(baked_vertices_.size()));
        return baked_vertices_[static_cast<size_t>(i)];
    }
    /// True if any baked vertex has a nonzero weight (sum > 1e-9) in any
    /// camera slot — semantic query replacing external loops.
    /// Precondition: IsBaked().
    bool HasVisibleVertices() const;

private:
    // Finite-difference normals: flat (z ~ 0) -> (0,0,1); curved -> unit normal
    // derived from dz/dx, dz/dy estimated from retained grid neighbours.
    static void ComputeBowlNormals(BowlModel& mesh,
                                   const std::vector<int>& grid_to_index,
                                   const BowlParams& params, int rows, int cols);

    std::vector<cv::Vec3d> vertices_;
    std::vector<cv::Vec3d> normals_;
    std::vector<std::array<int, 3>> triangles_;
    int rows_ = 0;
    int cols_ = 0;

    std::vector<BakedVertex> baked_vertices_;
    int baked_num_cameras_ = 0;
};

} // namespace avm_stitch
