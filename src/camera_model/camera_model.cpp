#include "camera_model/camera_model.hpp"

#include "camera_model/kb_camera_model.hpp"
#include "camera_model/lut_camera_model.hpp"
#include "camera_model/ocam_camera_model.hpp"

#include <algorithm>
#include <cmath>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

namespace avm_stitch {

CameraModel::CameraModel(int width, int height, double cx, double cy,
                         double fisheye_radius)
    : width_(width), height_(height), cx_(cx), cy_(cy),
      gate_radius_raw_(fisheye_radius) {}

// Single-point projection. Base default: route through the batch ProjectPoints
// (one-element spans). kb overrides with the θ-polynomial formula directly.
cv::Vec2d CameraModel::ProjectCameraPoint(const cv::Vec3d& P_cam) const {
    cv::Vec2d uv;
    uint8_t valid = 0;
    ProjectPoints(std::span<const cv::Vec3d>(&P_cam, 1),
                  std::span<cv::Vec2d>(&uv, 1),
                  std::span<uint8_t>(&valid, 1));
    return uv;
}

uint8_t CameraModel::RectContains(double u, double v) const {
    return (u >= 0.0 && u < static_cast<double>(width_) &&
            v >= 0.0 && v < static_cast<double>(height_))
               ? 1
               : 0;
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

// Fisheye effective-circle radius. 0 in the raw config -> auto: the inscribed
// circle centred at (cx,cy) (distance to the nearest image edge). Moved
// verbatim from the retired hand-written FisheyeCircleRadius.
double CameraModel::FisheyeCircleRadius() const {
    if (gate_radius_raw_ > 0.0)
        return gate_radius_raw_;
    double r = std::min({cx_, cy_,
                         static_cast<double>(width_) - cx_,
                         static_cast<double>(height_) - cy_});
    return std::max(0.0, r);
}

// True when the projected pixel lies inside the fisheye effective circle
// (valid image content; outside is the black dead zone). Moved verbatim from
// the retired hand-written InsideFisheyeCircle.
bool CameraModel::InsideFisheyeCircle(double u_px, double v_px) const {
    double r_px = std::hypot(u_px - cx_, v_px - cy_);
    return r_px <= FisheyeCircleRadius();
}

// Hard gate for pixels near the fisheye effective-circle edge: 1.0 well
// inside, 0.0 within the feather band. Prevents the dark vignette at the
// fisheye rim (measured luminance profile: r220=71, r230=19, r240=0.15)
// from being sampled — the "feather" acts as a hard visibility gate, not a
// soft fade (since the dark rim is pure black, linear interpolation cannot
// salvage it). feather_px is the exclusion width in pixels (<= 0 -> no
// exclusion). Default 40 px covers the full vignette ramp. Moved verbatim
// from the retired hand-written EdgeFeather.
double CameraModel::EdgeFeather(double u_px, double v_px,
                                double feather_px) const {
    if (feather_px <= 0.0)
        return 1.0;
    const double r_px = std::hypot(u_px - cx_, v_px - cy_);
    const double radius = FisheyeCircleRadius();
    const double margin = radius - r_px; // >0 guaranteed inside circle
    // Hard gate: exclude everything within feather_px of the circle edge
    // (was a linear fade before the 2026-09-11 fix — changed to a step
    // because the D3 fallback normalizes weight and amplifies even a
    // fractionally-safe sample to full weight, rendering the dark rim).
    if (margin >= feather_px)
        return 1.0;
    return 0.0;
}

std::unique_ptr<CameraModel> MakeCameraModel(const std::string& model,
                                             const CameraModelParams& params) {
    if (model == "kb") {
        if (!std::holds_alternative<KbParams>(params)) {
            throw std::invalid_argument(
                "MakeCameraModel: params for model 'kb' must be KbParams");
        }
        const auto& kb = std::get<KbParams>(params);
        if (kb.width <= 0 || kb.height <= 0) {
            throw std::invalid_argument(
                "MakeCameraModel: kb camera width/height must be > 0, got " +
                std::to_string(kb.width) + "x" + std::to_string(kb.height));
        }
        return std::make_unique<KbCameraModel>(kb);
    }
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
                    "strictly ascending (row " +
                    std::to_string(i) + ")");
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

}
