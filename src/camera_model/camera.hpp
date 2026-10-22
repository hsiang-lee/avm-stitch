#pragma once
// Camera value object: model travels with the lens, extrinsics with the
// calibration (spec D1). Extracted from camera_model.hpp into its own header
// so the base abstraction (CameraModel + factory params) and the pose/model
// bundle (Camera) stay individually includable.

#include "camera_model/camera_model.hpp"   // CameraModel + params/factory

#include <memory>
#include <span>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

namespace avm_stitch {

// Model travels with the lens, extrinsics with the calibration (spec D1).
// The model pointer is private: null-model checks go through Valid(),
// projection/dimensions through the delegating accessors below.
class Camera {
public:
    Camera() = default;                                        // no model: Valid() == false
    Camera(CameraExtrinsics ext, std::unique_ptr<CameraModel> model)
        : extrinsics(std::move(ext)), model_(std::move(model)) {}

    Camera(Camera&&) = default;
    Camera& operator=(Camera&&) = default;
    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;

    bool Valid() const { return model_ != nullptr; }

    // Precondition: Valid(). Width/height of the associated image.
    int Width() const { return model_->width(); }
    int Height() const { return model_->height(); }

    // Precondition: Valid(). Delegation to the underlying CameraModel
    // (bake-layer gating surface).
    double Cx() const { return model_->Cx(); }
    double Cy() const { return model_->Cy(); }
    bool InsideFisheyeCircle(double u, double v) const {
        return model_->InsideFisheyeCircle(u, v);
    }
    double EdgeFeather(double u, double v, double feather_px = 40.0) const {
        return model_->EdgeFeather(u, v, feather_px);
    }
    void ProjectPoints(std::span<const cv::Vec3d> cam_points,
                       std::span<cv::Vec2d> out_uv,
                       std::span<uint8_t> out_valid) const {
        model_->ProjectPoints(cam_points, out_uv, out_valid);
    }

    CameraExtrinsics extrinsics;   // pose value data stays public (no ownership hazard)

private:
    std::unique_ptr<CameraModel> model_;
};

} // namespace avm_stitch
