#pragma once

// Bowl model: car-parameterized bowl mesh generation + per-vertex bake
// (UV / blend weights), fused into construction — one geometry class owns
// both (pure math). Construction invariant: once built, baked data is
// always available (no half-baked state).
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

/// Bowl mesh type: BlockBoat = square grid + circular clipping (avm_renderer default),
/// BowlBoat = polar coordinates + car-base distance.
enum class BowlType { BlockBoat, BowlBoat };

struct BowlParams {
    double car_length = 5.6;
    double car_width = 2.2;
    double round_l = 2.0;
    double step = 0.2;
    double r_outter = 12.6;
    double k_slop = 0.09;
    double z_exp = 2.2;
    double max_z = 8.6;
    BowlType type = BowlType::BlockBoat;   // default = square grid (matches avm_renderer)
    cv::Vec2d car_center{0.0, 0.0};        // 新增: 车体中心在数据坐标系中的位置 (m)
                                           // 默认 (0,0) = 底部中心系，向后兼容
                                           // 后轴系: 调用方设 (axis_x/2, 0)

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
// per-vertex baked UV + blend weights. Mesh generation + bake are fused
// into construction (construction invariant: once built, baked data is
// always available). Immutable data holder: read via the fine-grained
// index accessors and semantic queries (the internal container layout is
// an implementation detail).
class BowlModel {
public:
    /// Generate the bowl mesh and bake per-vertex UV + blend weights in one
    /// step. Construction invariant: once built, baked data is always
    /// available.
    explicit BowlModel(const BowlParams& params,
                       const std::vector<CameraIntrinsics>& intrinsics,
                       const std::vector<CameraExtrinsics>& extrinsics);

    /// Wrap pre-built vertex data and bake immediately (tests / custom
    /// geometry): normals are taken as-is, no grid/normal recomputation,
    /// no triangles.
    /// @param car_center  sector pole passed to ComputeBlendWeights (default (0,0))
    BowlModel(std::vector<cv::Vec3d> vertices, std::vector<cv::Vec3d> normals,
              const std::vector<CameraIntrinsics>& intrinsics,
              const std::vector<CameraExtrinsics>& extrinsics,
              const cv::Vec2d& car_center = cv::Vec2d(0.0, 0.0));

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

    // --- baked per-vertex UV + blend weights (construction invariant) ---
    /// Number of cameras the model was baked for at construction.
    int baked_num_cameras() const {
        assert(baked_num_cameras_ > 0);
        return baked_num_cameras_;
    }
    /// Baked vertex i (one per bowl vertex).
    /// Precondition: 0 <= i < vertex_count().
    const BakedVertex& baked_vertex(int i) const {
        assert(i >= 0 && i < static_cast<int>(baked_vertices_.size()));
        return baked_vertices_[static_cast<size_t>(i)];
    }
    /// True if any baked vertex has a nonzero weight (sum > 1e-9) in any
    /// camera slot — semantic query replacing external loops.
    bool HasVisibleVertices() const;

    /// Compute rd (distance to rounded-rectangle flat region) for a point.
    static double ComputeRd(double x, double y, const BowlParams& params);
    /// Compute rd for BowlBoat: 9-region distance to car rectangle (S1-S9).
    static double ComputeRdBowlBoat(double x, double y, const BowlParams& params);

private:
    // Square grid mesh (create_block_boat_model from avm_renderer).
    void GenerateBlockBoatMesh(const BowlParams& params);
    // Polar coordinate mesh (create_bowl_boat_model from avm_renderer).
    void GenerateBowlBoatMesh(const BowlParams& params);

    // Finite-difference normals: flat (z ~ 0) -> (0,0,1); curved -> unit normal
    // derived from dz/dx, dz/dy estimated from retained grid neighbours.
    static void ComputeBowlNormals(BowlModel& mesh,
                                   const std::vector<int>& grid_to_index,
                                   const BowlParams& params, int rows, int cols);

    // Per-vertex bake, called once at the end of each constructor.
    // car_center is the sector pole passed to ComputeBlendWeights (default (0,0)).
    void BakeVertices(const std::vector<CameraIntrinsics>& intrinsics,
                      const std::vector<CameraExtrinsics>& extrinsics,
                      const cv::Vec2d& car_center = cv::Vec2d(0.0, 0.0));

    std::vector<cv::Vec3d> vertices_;
    std::vector<cv::Vec3d> normals_;
    std::vector<std::array<int, 3>> triangles_;
    int rows_ = 0;
    int cols_ = 0;

    std::vector<BakedVertex> baked_vertices_;
    int baked_num_cameras_ = 0;
};

} // namespace avm_stitch
