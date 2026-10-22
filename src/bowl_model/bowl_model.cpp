#include "bowl_model/bowl_model.hpp"

#include "bowl_model/blend_weights.hpp"

#include <span>
#include <vector>

namespace avm_stitch {

BowlModel::BowlModel(const BowlParams& params,
                     const std::vector<Camera>& cameras) {
    params.GenerateBlockBoatMesh(vertices_, normals_, triangles_);
    baked_vertices_ = BakeVertices(cameras, params.CarCenter());
}

std::vector<BowlModel::BakedVertex>
BowlModel::BakeVertices(const std::vector<Camera>& cameras,
                        const cv::Vec2d& car_center) {
    const int num_cameras = static_cast<int>(cameras.size());
    const int num_vertices = static_cast<int>(vertices_.size());
    std::vector<BakedVertex> baked_vertices(static_cast<size_t>(num_vertices),
                                            BakedVertex(num_cameras));
    const BlendWeightsCalculator blend_calculator(cameras, car_center);
    std::vector<double> edge_factors(static_cast<size_t>(num_cameras), 0.0);

    for (int vertex_index = 0; vertex_index < num_vertices; ++vertex_index) {
        const cv::Vec3d& world_position = vertices_[static_cast<size_t>(vertex_index)];
        const auto blend_weights = blend_calculator.WeightsFor(world_position);
        BakedVertex& baked_vertex = baked_vertices[static_cast<size_t>(vertex_index)];
        for (int camera_index = 0; camera_index < num_cameras; ++camera_index) {
            const Camera& camera = cameras[static_cast<size_t>(camera_index)];
            const cv::Vec3d camera_point = camera.Extrinsics().ToCamera(world_position);
            cv::Vec2d pixel_uv;
            uint8_t inside_image_rect = 0;
            camera.ProjectPoints(std::span(&camera_point, 1), std::span(&pixel_uv, 1),
                                 std::span(&inside_image_rect, 1));
            double edge_factor = 0.0;
            if (inside_image_rect != 0 &&
                camera.InsideFisheyeCircle(pixel_uv[0], pixel_uv[1])) {
                baked_vertex.uv[static_cast<size_t>(camera_index)][0] =
                    static_cast<float>(pixel_uv[0] / camera.Width());
                baked_vertex.uv[static_cast<size_t>(camera_index)][1] =
                    static_cast<float>(pixel_uv[1] / camera.Height());
                edge_factor = camera.EdgeFeather(pixel_uv[0], pixel_uv[1]);
            }
            edge_factors[static_cast<size_t>(camera_index)] = edge_factor;
        }
        blend_calculator.NormalizeWeights(blend_weights, edge_factors,
                                          baked_vertex.weight);
    }
    return baked_vertices;
}

bool BowlModel::HasVisibleVertices() const {
    const int num_vertices = vertex_count();
    const int num_cameras = baked_num_cameras();
    for (int vertex_index = 0; vertex_index < num_vertices; ++vertex_index) {
        const BakedVertex& baked_vertex = baked_vertices_[vertex_index];
        double weight_sum = 0.0;
        for (int camera_index = 0; camera_index < num_cameras; ++camera_index)
            weight_sum += baked_vertex.weight[camera_index];
        if (weight_sum > 1e-9)
            return true;
    }
    return false;
}

} // namespace avm_stitch
