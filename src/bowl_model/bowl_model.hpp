#pragma once

#include "bowl_model/bowl_params.hpp"
#include "camera_model/camera.hpp"
#include "camera_model/camera_extrinsics.hpp"

#include <opencv2/core.hpp>

#include <array>
#include <cassert>
#include <vector>

namespace avm_stitch {

class BowlModel {
  public:
    struct BakedVertex {
        std::vector<std::array<float, 2>> uv;
        std::vector<float> weight;

        explicit BakedVertex(int num_cameras)
            : uv(static_cast<size_t>(num_cameras), {0.0f, 0.0f}),
              weight(static_cast<size_t>(num_cameras), 0.0f) {}
    };
    explicit BowlModel(const BowlParams& params,
                       const std::vector<Camera>& cameras);

    int vertex_count() const { return static_cast<int>(vertices_.size()); }
    const cv::Vec3d& vertex(int i) const {
        assert(i >= 0 && i < vertex_count());
        return vertices_[static_cast<size_t>(i)];
    }
    int normal_count() const { return static_cast<int>(normals_.size()); }
    const cv::Vec3d& normal(int i) const {
        assert(i >= 0 && i < normal_count());
        return normals_[static_cast<size_t>(i)];
    }
    int triangle_count() const { return static_cast<int>(triangles_.size()); }
    std::array<int, 3> triangle(int i) const {
        assert(i >= 0 && i < triangle_count());
        return triangles_[static_cast<size_t>(i)];
    }
    int baked_num_cameras() const {
        return baked_vertices_.empty()
                   ? 0
                   : static_cast<int>(baked_vertices_[0].weight.size());
    }
    const BakedVertex& baked_vertex(int i) const {
        assert(i >= 0 && i < static_cast<int>(baked_vertices_.size()));
        return baked_vertices_[static_cast<size_t>(i)];
    }
    bool HasVisibleVertices() const;

  private:
    std::vector<BakedVertex> BakeVertices(const std::vector<Camera>& cameras,
                                          const cv::Vec2d& car_center);

    std::vector<cv::Vec3d> vertices_;
    std::vector<cv::Vec3d> normals_;
    std::vector<std::array<int, 3>> triangles_;

    std::vector<BakedVertex> baked_vertices_;
};

} // namespace avm_stitch
