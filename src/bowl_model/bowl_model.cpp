#include "bowl_model/bowl_model.hpp"

#include "bowl_model/blend_weights.hpp"

#include <stdexcept>
#include <vector>

namespace avm_stitch {

namespace {

void NormalizeWeights(int n,
                      const std::vector<double>& raw_weights,
                      const std::vector<char>& edge_ok,
                      const std::vector<double>& edge_factor,
                      BowlModel::BakedVertex& bv) {
    double visible_contrib_sum = 0.0;
    double edge_only_sum = 0.0;
    for (int j = 0; j < n; ++j) {
        if (edge_ok[j]) {
            if (raw_weights[j] > 1e-9) visible_contrib_sum += raw_weights[j] * edge_factor[j];
            edge_only_sum += edge_factor[j];
        }
    }

    if (visible_contrib_sum > 1e-9) {
                for (int j = 0; j < n; ++j) {
            if (edge_ok[j] && raw_weights[j] > 1e-9) {
                bv.weight[j] = static_cast<float>(
                    raw_weights[j] * edge_factor[j] / visible_contrib_sum);
            }
        }
    } else if (edge_only_sum > 1e-9) {
                                for (int j = 0; j < n; ++j) {
            if (edge_ok[j]) {
                bv.weight[j] = static_cast<float>(edge_factor[j] / edge_only_sum);
            }
        }
    }
    }

}
BowlModel::BowlModel(const BowlParams& params, const std::vector<Camera>& cameras) {
    params.GenerateBlockBoatMesh(vertices_, normals_, triangles_);
    baked_vertices_ = BakeVertices(cameras, params.car_center());
    baked_num_cameras_ = static_cast<int>(cameras.size());
}

std::vector<BowlModel::BakedVertex> BowlModel::BakeVertices(
    const std::vector<Camera>& cameras,
    const cv::Vec2d& car_center) {
    const int N = static_cast<int>(cameras.size());
    for (int j = 0; j < N; ++j) {
        if (!cameras[j].Valid()) {
            throw std::invalid_argument(
                "BowlModel: camera " + std::to_string(j) +
                " has no model (Camera.model is null)");
        }
    }
    const int vertex_count = static_cast<int>(vertices_.size());
    std::vector<BakedVertex> baked(vertices_.size(), BakedVertex(N));

                std::vector<std::vector<double>> edge_factors(
        static_cast<size_t>(N), std::vector<double>(vertex_count, 0.0));

            std::vector<cv::Vec3d> cam_points(vertex_count);
    std::vector<cv::Vec2d> uv(vertex_count);
    std::vector<uint8_t> valid(vertex_count);

    for (int j = 0; j < N; ++j) {
        const auto& cam = cameras[j];
        const CameraExtrinsics& extr = cam.extrinsics;
        for (int i = 0; i < vertex_count; ++i) {
            cam_points[i] = extr.ToCamera(vertices_[i]);
        }
        cam.ProjectPoints(cam_points, uv, valid);

        auto& factors = edge_factors[static_cast<size_t>(j)];
        for (int i = 0; i < vertex_count; ++i) {
            auto& bv = baked[i];
            const double u_px = uv[i][0];
            const double v_px = uv[i][1];

                                                                                                const bool in_circle = valid[i] != 0
                                   && cam.InsideFisheyeCircle(u_px, v_px);
            if (in_circle) {
                bv.uv[j][0] = static_cast<float>(u_px / cam.Width());
                bv.uv[j][1] = static_cast<float>(v_px / cam.Height());
                factors[static_cast<size_t>(i)] = cam.EdgeFeather(u_px, v_px);
            }
        }
    }

            const auto extrinsics = ExtractExtrinsics(cameras);
    for (int i = 0; i < vertex_count; ++i) {
        const cv::Vec3d& P = vertices_[i];
        const auto raw_weights = ComputeBlendWeights(P, extrinsics, car_center);

        std::vector<char> edge_ok(N, 0);
        std::vector<double> edge_factor(N, 0.0);
        for (int j = 0; j < N; ++j) {
            edge_factor[j] = edge_factors[static_cast<size_t>(j)][static_cast<size_t>(i)];
            edge_ok[j] = edge_factor[j] > 0.0;
        }
        NormalizeWeights(N, raw_weights, edge_ok, edge_factor, baked[i]);
    }
    return baked;
}

std::vector<CameraExtrinsics> BowlModel::ExtractExtrinsics(
    const std::vector<Camera>& cameras) {
    std::vector<CameraExtrinsics> extrs;
    extrs.reserve(cameras.size());
    for (const auto& cam : cameras) extrs.push_back(cam.extrinsics);
    return extrs;
}

bool BowlModel::HasVisibleVertices() const {
                const int n = vertex_count();
    const int cams = baked_num_cameras_;
    for (int i = 0; i < n; ++i) {
        const BakedVertex& bv = baked_vertices_[i];
        double sum = 0.0;
        for (int j = 0; j < cams; ++j) sum += bv.weight[j];
        if (sum > 1e-9) return true;
    }
    return false;
}

}
