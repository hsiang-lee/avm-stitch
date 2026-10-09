#pragma once

// OcamCameraModel: Scaramuzza omnidirectional model (Representation C,
// renderer-parity with avm_mapper::OcamModel, ocam_model.hpp:111-131).
// Sign convention: avm-stitch camera frame is OpenCV (+Z into the scene)
// while the OCAM front hemisphere is Z<0, so the implementation consumes the
// flipped ray (x, y, -z). Points behind the camera (z <= 0) are outside the
// model's domain: valid = 0.

#include "geometry/camera_model.hpp"

namespace avm_stitch {

class OcamCameraModel final : public CameraModel {
public:
    OcamCameraModel(std::vector<double> pol, std::vector<double> invpol,
                    double cx, double cy, double c, double d, double e,
                    int width, int height);

    void ProjectPoints(std::span<const cv::Vec3d> cam_points,
                       std::span<cv::Vec2d> out_uv,
                       std::span<uint8_t> out_valid) const override;

private:
    std::vector<double> pol_;     // cam2world (low -> high), kept for tests
    std::vector<double> invpol_;  // world2cam (low -> high)
    double c_, d_, e_;            // affine stretch A = [[c, d], [e, 1]]
};

} // namespace avm_stitch
