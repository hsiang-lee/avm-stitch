#pragma once

// Two-phase AVM stitch engine API:
//   Phase 1 Configure(...) — one-time setup.
//     Builds the bowl model, bakes per-vertex UV + blend weights, builds the
//     OSG scene graph and prepares the offscreen renderer. GL-free.
//     Two overloads:
//       - Configure(intrinsics, extrinsics, ...) — legacy kb-only wrapper.
//       - Configure(cameras, ...) — multi-model path (spec D1): each Camera
//         carries its own CameraModel (kb today, lut/ocam in batch B).
//   Phase 2 RenderFrame(images[N]) — per frame.
//     Uploads the N fisheye images into the camera texture array and renders
//     the 4 fixed side views (3d_front, 3d_rear, 3d_left, 3d_right).
// PIMPL hides all OSG headers from callers.

#include "geometry/bowl_model.hpp"
#include "geometry/camera_model.hpp"
#include "geometry/projection.hpp"

#include <opencv2/core.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace avm_stitch {

class StitchEngine {
public:
    StitchEngine();
    ~StitchEngine();
    StitchEngine(const StitchEngine&) = delete;
    StitchEngine& operator=(const StitchEngine&) = delete;

    /// Configure the engine (one-time). BowlParams carries car dimensions.
    /// @param intrinsics   N camera intrinsics (1 <= N <= 8)
    /// @param extrinsics   N camera extrinsics T_wc (camera -> world)
    /// @param bowl_params  bowl model parameters (incl. car length/width)
    /// @param output_width  output image width (default 960)
    /// @param output_height output image height (default 640)
    /// @throws std::invalid_argument if N=0, N>8, or sizes mismatch
    void Configure(const std::vector<CameraIntrinsics>& intrinsics,
                   const std::vector<CameraExtrinsics>& extrinsics,
                   const BowlParams& bowl_params,
                   int output_width = 960,
                   int output_height = 640);

    /// Configure the engine from camera models (multi-model path, spec D1).
    /// @throws std::invalid_argument if N=0, N>8, any Camera.model is null,
    ///         or camera image sizes mismatch
    void Configure(const std::vector<Camera>& cameras,
                   const BowlParams& bowl_params,
                   int output_width = 960,
                   int output_height = 640);

    /// Render one frame from N fisheye images (BGR888, must match Configure N).
    /// @return {"3d_front": Mat, "3d_rear": Mat, ...} — 4 views, BGR888
    /// @throws std::runtime_error if not configured
    /// @throws std::invalid_argument if images.size() != N
    std::map<std::string, cv::Mat> RenderFrame(
        const std::vector<cv::Mat>& images) const;

    bool IsConfigured() const;
    int GetNumCameras() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace avm_stitch
