#pragma once

// Configures N calibrated cameras and runs the full pipeline inline —
// OSG scene graph + offscreen renderer. GL-free.

#include "bowl_model/bowl_model.hpp"
#include "camera_model/camera.hpp"

#include <opencv2/core.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace avm_stitch {

class StitchScene;
class OSGViewRenderer;

class StitchEngine {
  public:
    StitchEngine();
    ~StitchEngine();
    StitchEngine(const StitchEngine&) = delete;
    StitchEngine& operator=(const StitchEngine&) = delete;

    // N calibrated Camera objects (moved in); each Camera carries its model
    // and extrinsics as one unit. Ownership of every model transfers to the
    // engine for the lifetime of the pipeline.
    void Configure(std::vector<Camera> cameras,
                   const BowlParams& bowl_params,
                   int output_width = 960,
                   int output_height = 640);

    std::map<std::string, cv::Mat> RenderFrame(
        const std::vector<cv::Mat>& images) const;

    bool IsConfigured() const;
    int GetNumCameras() const;

  private:
    std::unique_ptr<StitchScene> scene_;
    std::unique_ptr<OSGViewRenderer> renderer_;
    bool configured_ = false;
};

}
