#pragma once

// Two-phase AVM stitch engine API:
//   Phase 1 Configure(intrinsics, extrinsics, bowl_params) — one-time setup.
//     Builds the bowl mesh, bakes per-vertex UV + blend weights, builds the
//     OSG scene graph and prepares the offscreen renderer. GL-free.
//   Phase 2 RenderFrame(images[N]) — per frame.
//     Uploads the N fisheye images into the camera texture array and renders
//     the 5 fixed views (topdown, 3d_front, 3d_rear, 3d_left, 3d_right).
// PIMPL hides all OSG headers from callers.

#include "avm_stitch/bowl_model.hpp"
#include "avm_stitch/projection.hpp"

#include <opencv2/core.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace avm_worker { namespace avm_stitch {

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

    /// Render one frame from N fisheye images (BGR888, must match Configure N).
    /// @return {"topdown": Mat, "3d_front": Mat, ...} — 5 views, BGR888
    /// @throws std::runtime_error if not configured
    /// @throws std::invalid_argument if images.size() != N
    std::map<std::string, cv::Mat> RenderFrame(
        const std::vector<cv::Mat>& images) const;

    bool IsConfigured() const;
    int GetNumCameras() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    bool configured_ = false;
    int num_cameras_ = 0;
};

}} // namespace avm_worker::avm_stitch
