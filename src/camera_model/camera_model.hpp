#pragma once
// The Camera value object (pose + model bundle) lives in camera.hpp.

#include "camera_model/camera_extrinsics.hpp"
#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <variant>
#include <vector>

#include <opencv2/core.hpp>

namespace avm_stitch {

// Numerical epsilon shared by equidistant projections (formerly a member of
// the retired hand-written intrinsic model, camera_extrinsics.hpp).
// OcamCameraModel uses its own 1e-12 literal — untouched.
constexpr double kEps = 1e-9;

class CameraModel {
public:
    virtual ~CameraModel() = default;

                        virtual void ProjectPoints(std::span<const cv::Vec3d> cam_points,
                               std::span<cv::Vec2d> out_uv,
                               std::span<uint8_t> out_valid) const = 0;

    // Single-point forward projection (camera-frame point -> pixel). The base
    // default routes through ProjectPoints (one-element spans); kb overrides it
    // with the θ-polynomial so the batch loop and single-point queries share
    // exactly the same formula.
    virtual cv::Vec2d ProjectCameraPoint(const cv::Vec3d& P_cam) const;

    int width() const { return width_; }
    int height() const { return height_; }
    virtual double Cx() const { return cx_; }
    virtual double Cy() const { return cy_; }

        // Implemented on the base directly (the retired hand-written intrinsic
    // model's gating methods moved verbatim), seeded once in the ctor from
    // the subclass-
    // provided geometry + raw fisheye_radius (0 = auto inscribed circle). The
    // bake layer consumes these instead of rebuilding a per-camera intrinsics
    // object — the model IS the single gating source. D3 history: the feather
    // is a hard gate (the dark rim is pure black, interpolation cannot salvage
    // it).
    double FisheyeCircleRadius() const;
    bool InsideFisheyeCircle(double u, double v) const;
    double EdgeFeather(double u, double v, double feather_px = 40.0) const;

protected:
    // fisheye_radius: raw configured effective-circle radius for the gating
        CameraModel(int width, int height, double cx, double cy,
                double fisheye_radius = 0.0);

            uint8_t RectContains(double u, double v) const;

            static void RequireEqualLengths(std::span<const cv::Vec3d> cam_points,
                                    std::span<cv::Vec2d> out_uv,
                                    std::span<uint8_t> out_valid);

    int width_;
    int height_;
    double cx_;
    double cy_;

private:
    // Raw configured effective-circle radius (0 = auto inscribed circle).
    // The gating formulas are the base's own implementations (moved verbatim
    // from the retired hand-written intrinsic model); no per-frame reassembly.
    double gate_radius_raw_;
};

// Flat KbParams: image geometry + k1..k4 θ-polynomial coefficients +
// fisheye_radius (0 = auto inscribed circle). The retired hand-written
// intrinsic model folded into this struct; the projection polynomial lives
// in KbCameraModel::ProjectCameraPoint (the single source of the formula).
struct KbParams {
    int width = 0;
    int height = 0;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    double k1 = 0.0;
    double k2 = 0.0;
    double k3 = 0.0;
    double k4 = 0.0;
    double fisheye_radius = 0.0;
};
struct LutParams  {
    std::vector<std::array<double, 2>> points;      double pixel_size_mm = 0.003;                   int width = 0, height = 0;
    double cx = 0.0, cy = 0.0;
};
struct OcamParams {
    std::vector<double> pol;                        std::vector<double> invpol;                     double cx = 0.0, cy = 0.0;
    double c = 1.0, d = 0.0, e = 0.0;
    int width = 0, height = 0;
};
using CameraModelParams = std::variant<KbParams, LutParams, OcamParams>;

std::unique_ptr<CameraModel> MakeCameraModel(const std::string& model,
                                             const CameraModelParams& params);

}
