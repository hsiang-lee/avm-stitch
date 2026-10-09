#pragma once

// LutCameraModel: lookup-table fisheye (Representation A, renderer-parity).
// The table maps incidence angle (deg) -> image-plane radius (mm) on the
// sensor; forward projection looks the angle up DIRECTLY — no numerical
// inversion (renderer LensDistortionTable::radiusPxForAngle,
// lens_distortion_table.hpp:122-128).

#include "geometry/camera_model.hpp"

namespace avm_stitch {

class LutCameraModel final : public CameraModel {
public:
    LutCameraModel(std::vector<std::array<double, 2>> points,
                   double pixel_size_mm, int width, int height,
                   double cx, double cy);

    void ProjectPoints(std::span<const cv::Vec3d> cam_points,
                       std::span<cv::Vec2d> out_uv,
                       std::span<uint8_t> out_valid) const override;

private:
    // Image-plane radius in px for an incidence angle (deg); endpoint clamp.
    double RadiusPxForAngle(double theta_deg) const;

    std::vector<std::array<double, 2>> points_;   // (angle_deg, height_mm) ascending
    double pixel_size_mm_;
};

} // namespace avm_stitch
