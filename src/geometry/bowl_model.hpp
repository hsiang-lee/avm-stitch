#pragma once

// Bowl model: car-parameterized bowl mesh generation (pure math).
// Algorithm mirrors the reference `create_block_boat_model` (bowlModel.hpp):
//   z = k_slop * rd^z_exp outside the rounded-rectangle flat region,
//   finite-difference normals on the curved rim.
// Mesh generation lives in bowl_model.cpp; only lightweight accessors are
// inline here.

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

// Bowl surface triangle mesh generated from car parameters. Immutable data
// holder: read via the fine-grained index accessors (the internal container
// layout is an implementation detail, free to change e.g. to SoA).
class BowlMesh {
public:
    /// Generate the bowl mesh (VCS coordinates) from car parameters.
    explicit BowlMesh(const BowlParams& params);

    /// Wrap pre-built vertex data (for tests / custom geometry): normals are
    /// taken as-is, no grid/normal recomputation, no triangles.
    BowlMesh(std::vector<cv::Vec3d> vertices, std::vector<cv::Vec3d> normals);

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

private:
    // Finite-difference normals: flat (z ~ 0) -> (0,0,1); curved -> unit normal
    // derived from dz/dx, dz/dy estimated from retained grid neighbours.
    static void ComputeBowlNormals(BowlMesh& mesh,
                                   const std::vector<int>& grid_to_index,
                                   const BowlParams& params, int rows, int cols);

    std::vector<cv::Vec3d> vertices_;
    std::vector<cv::Vec3d> normals_;
    std::vector<std::array<int, 3>> triangles_;
    int rows_ = 0;
    int cols_ = 0;
};

} // namespace avm_stitch
