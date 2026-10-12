#pragma once
// Camera model abstraction (spec D1): batch forward projection only.
// Pure geometry — no IO / transport decoding (spec §8: base64/CSV/文件束归 worker).

#include "geometry/projection.hpp"   // CameraIntrinsics / CameraExtrinsics

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <variant>
#include <vector>

#include <opencv2/core.hpp>

namespace avm_stitch {

// Batch forward projection: camera-frame points -> pixels + per-point validity.
// One virtual call per camera per frame (hot-loop constraint, spec D1 3.3).
class CameraModel {
public:
    virtual ~CameraModel() = default;

    // out_uv[i] / out_valid[i] correspond to cam_points[i]. Mismatched span
    // lengths -> throw std::invalid_argument.
    // out_valid[i] = 1 iff the projected pixel lies in [0,width) x [0,height).
    // Content-level gating (fisheye dead zone / feather) stays in the bake
    // layer via FisheyeRadius/Cx/Cy below — NOT here (kb 回归不变).
    virtual void ProjectPoints(std::span<const cv::Vec3d> cam_points,
                               std::span<cv::Vec2d> out_uv,
                               std::span<uint8_t> out_valid) const = 0;

    int width() const { return width_; }
    int height() const { return height_; }
    // Fisheye effective-circle radius in px; 0 = auto (inscribed circle of
    // the image). kb forwards CameraIntrinsics::fisheye_radius; lut/ocam
    // default 0 (auto).
    virtual double FisheyeRadius() const { return 0.0; }
    virtual double Cx() const { return cx_; }
    virtual double Cy() const { return cy_; }

protected:
    CameraModel(int width, int height, double cx, double cy);

    // 1 iff the pixel lies in [0,width) x [0,height) — the shared
    // out_valid gate (base-class contract, all models).
    uint8_t RectContains(double u, double v) const;

    // Throws std::invalid_argument when the three spans differ in length
    // (shared ProjectPoints precondition of the base-class contract).
    static void RequireEqualLengths(std::span<const cv::Vec3d> cam_points,
                                    std::span<cv::Vec2d> out_uv,
                                    std::span<uint8_t> out_valid);

    int width_;
    int height_;
    double cx_;
    double cy_;
};

// Model travels with the lens, extrinsics with the calibration (spec D1).
struct Camera {
    CameraExtrinsics extrinsics;
    std::unique_ptr<CameraModel> model;
};

// ── Factory params (variant prevents cross-model field mixing) ──
struct KbParams   { CameraIntrinsics intr; };   // k1..k4 = θ 多项式系数
struct LutParams  {
    std::vector<std::array<double, 2>> points;  // (angle_deg, height_mm) 必须严格升序
    double pixel_size_mm = 0.003;               // 与 renderer LensDistortionTable 默认一致
    int width = 0, height = 0;
    double cx = 0.0, cy = 0.0;
};
struct OcamParams {
    std::vector<double> pol;                    // cam2world 多项式，低→高次
    std::vector<double> invpol;                 // world2cam 多项式，低→高次
    double cx = 0.0, cy = 0.0;
    double c = 1.0, d = 0.0, e = 0.0;
    int width = 0, height = 0;
};
using CameraModelParams = std::variant<KbParams, LutParams, OcamParams>;

// Vocabulary: kb | table | ocam (renderer-parity; table ↔ undistort="lookup").
// Throws std::invalid_argument on: unknown model string; variant type not
// matching the model; invalid params (LUT <2 points or not ascending,
// empty pol/invpol, width/height <= 0).
std::unique_ptr<CameraModel> MakeCameraModel(const std::string& model,
                                             const CameraModelParams& params);

} // namespace avm_stitch
