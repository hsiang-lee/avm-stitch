#pragma once

// KbCameraModel: equidistant θ-polynomial fisheye (k1..k4) — the multi-model
// home of the legacy ProjectWorldToFishEye math (Task A2 migrates it here,
// numerically equivalent, projection.hpp stays untouched).

#include "geometry/camera_model.hpp"

namespace avm_stitch {

class KbCameraModel final : public CameraModel {
public:
    explicit KbCameraModel(const CameraIntrinsics& intr);

    // RED (Task A2): ProjectPoints lands with the failing numerical-
    // equivalence tests; FisheyeRadius forwards intr.fisheye_radius.
    void ProjectPoints(std::span<const cv::Vec3d> cam_points,
                       std::span<cv::Vec2d> out_uv,
                       std::span<uint8_t> out_valid) const override;

    double FisheyeRadius() const override { return intr_.fisheye_radius; }

private:
    CameraIntrinsics intr_;
};

} // namespace avm_stitch
