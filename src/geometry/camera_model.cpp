#include "geometry/camera_model.hpp"

#include "geometry/kb_camera_model.hpp"
#include "geometry/lut_camera_model.hpp"
#include "geometry/ocam_camera_model.hpp"

#include <span>
#include <stdexcept>
#include <string>
#include <utility>

namespace avm_stitch {

CameraModel::CameraModel(int width, int height, double cx, double cy)
    : width_(width), height_(height), cx_(cx), cy_(cy) {}

uint8_t CameraModel::RectContains(double u, double v) const {
    return (u >= 0.0 && u < static_cast<double>(width_) &&
            v >= 0.0 && v < static_cast<double>(height_)) ? 1 : 0;
}

void CameraModel::RequireEqualLengths(std::span<const cv::Vec3d> cam_points,
                                      std::span<cv::Vec2d> out_uv,
                                      std::span<uint8_t> out_valid) {
    if (cam_points.size() != out_uv.size() ||
        cam_points.size() != out_valid.size()) {
        throw std::invalid_argument(
            "ProjectPoints: cam_points/out_uv/out_valid must have equal "
            "lengths");
    }
}

std::unique_ptr<CameraModel> MakeCameraModel(const std::string& model,
                                             const CameraModelParams& params) {
    if (model == "kb") {
        if (!std::holds_alternative<KbParams>(params)) {
            throw std::invalid_argument(
                "MakeCameraModel: params for model 'kb' must be KbParams");
        }
        const auto& intr = std::get<KbParams>(params).intr;
        if (intr.width <= 0 || intr.height <= 0) {
            throw std::invalid_argument(
                "MakeCameraModel: kb camera width/height must be > 0, got " +
                std::to_string(intr.width) + "x" + std::to_string(intr.height));
        }
        return std::make_unique<KbCameraModel>(intr);
    }
    // Task B2 adds ascending-order validation for the table branch.
    if (model == "table") {
        if (!std::holds_alternative<LutParams>(params)) {
            throw std::invalid_argument(
                "MakeCameraModel: params for model 'table' must be LutParams");
        }
        const auto& lut = std::get<LutParams>(params);
        if (lut.points.size() < 2) {
            throw std::invalid_argument(
                "MakeCameraModel: table LUT needs >= 2 points, got " +
                std::to_string(lut.points.size()));
        }
        for (size_t i = 1; i < lut.points.size(); ++i) {
            if (!(lut.points[i - 1][0] < lut.points[i][0])) {
                throw std::invalid_argument(
                    "MakeCameraModel: table LUT angle_deg column must be "
                    "strictly ascending (row " + std::to_string(i) + ")");
            }
        }
        if (!(lut.pixel_size_mm > 0.0)) {
            throw std::invalid_argument(
                "MakeCameraModel: table pixel_size_mm must be > 0, got " +
                std::to_string(lut.pixel_size_mm));
        }
        if (lut.width <= 0 || lut.height <= 0) {
            throw std::invalid_argument(
                "MakeCameraModel: table camera width/height must be > 0, got " +
                std::to_string(lut.width) + "x" + std::to_string(lut.height));
        }
        return std::make_unique<LutCameraModel>(
            lut.points, lut.pixel_size_mm, lut.width, lut.height, lut.cx, lut.cy);
    }
    if (model == "ocam") {
        if (!std::holds_alternative<OcamParams>(params)) {
            throw std::invalid_argument(
                "MakeCameraModel: params for model 'ocam' must be OcamParams");
        }
        const auto& ocam = std::get<OcamParams>(params);
        if (ocam.pol.empty() || ocam.invpol.empty()) {
            throw std::invalid_argument(
                "MakeCameraModel: ocam pol/invpol must be non-empty");
        }
        if (ocam.width <= 0 || ocam.height <= 0) {
            throw std::invalid_argument(
                "MakeCameraModel: ocam camera width/height must be > 0, got " +
                std::to_string(ocam.width) + "x" + std::to_string(ocam.height));
        }
        return std::make_unique<OcamCameraModel>(
            ocam.pol, ocam.invpol, ocam.cx, ocam.cy, ocam.c, ocam.d, ocam.e,
            ocam.width, ocam.height);
    }
    throw std::invalid_argument("MakeCameraModel: unknown model '" + model + "'");
}

} // namespace avm_stitch
