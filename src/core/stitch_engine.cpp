#include "core/stitch_engine.hpp"

#include "geometry/blend_weights.hpp"
#include "render/stitch_scene.hpp"
#include "render/osg_view_renderer.hpp"

#include <stdexcept>
#include <string>

namespace avm_stitch {

struct StitchEngine::Impl {
    std::unique_ptr<StitchScene> scene;
    std::unique_ptr<OSGViewRenderer> renderer;
    bool configured = false;
    int num_cameras = 0;

    /// Validate Configure arguments (N range, size match, shared image size,
    /// bowl params). Throws std::invalid_argument on the first violation.
    void ValidateConfigureArgs(
        const std::vector<CameraIntrinsics>& intrinsics,
        const std::vector<CameraExtrinsics>& extrinsics,
        const BowlParams& bowl_params) const {
        const int N = static_cast<int>(intrinsics.size());
        if (N == 0 || N > 8) {
            throw std::invalid_argument(
                "StitchEngine::Configure: N must be in [1, 8], got " + std::to_string(N));
        }
        if (extrinsics.size() != static_cast<size_t>(N)) {
            throw std::invalid_argument(
                "StitchEngine::Configure: intrinsics and extrinsics size mismatch");
        }
        // The camera texture array is one shared 2D array sized from intrinsics[0];
        // every camera image must match it, so reject heterogeneous image sizes.
        for (int i = 1; i < N; ++i) {
            if (intrinsics[i].width != intrinsics[0].width ||
                intrinsics[i].height != intrinsics[0].height) {
                throw std::invalid_argument(
                    "StitchEngine::Configure: all cameras must share the same image "
                    "size; camera 0 is " + std::to_string(intrinsics[0].width) + "x" +
                    std::to_string(intrinsics[0].height) + ", camera " + std::to_string(i) +
                    " is " + std::to_string(intrinsics[i].width) + "x" +
                    std::to_string(intrinsics[i].height));
            }
        }
        // Degenerate bowl params would produce an empty or exploding mesh grid
        // (rows = r_outter / step): reject them with a clear error instead of UB.
        if (bowl_params.step <= 0.0) {
            throw std::invalid_argument(
                "StitchEngine::Configure: bowl_params.step must be > 0, got " +
                std::to_string(bowl_params.step));
        }
        if (bowl_params.r_outter <= 0.0) {
            throw std::invalid_argument(
                "StitchEngine::Configure: bowl_params.r_outter must be > 0, got " +
                std::to_string(bowl_params.r_outter));
        }
        if (bowl_params.k_slop < 0.0) {
            throw std::invalid_argument(
                "StitchEngine::Configure: bowl_params.k_slop must be >= 0, got " +
                std::to_string(bowl_params.k_slop));
        }
        if (bowl_params.z_exp < 0.0) {
            throw std::invalid_argument(
                "StitchEngine::Configure: bowl_params.z_exp must be >= 0, got " +
                std::to_string(bowl_params.z_exp));
        }
    }

    void Configure(const std::vector<CameraIntrinsics>& intrinsics,
                   const std::vector<CameraExtrinsics>& extrinsics,
                   const BowlParams& bowl_params,
                   int output_width, int output_height) {
        ValidateConfigureArgs(intrinsics, extrinsics, bowl_params);
        const int N = static_cast<int>(intrinsics.size());

        // Phase 1 pipeline: bowl model + bake UV/weights -> OSG scene graph.
        const BowlModel model(bowl_params, intrinsics, extrinsics);

        const int img_w = intrinsics[0].width;
        const int img_h = intrinsics[0].height;
        scene = std::make_unique<StitchScene>(model, img_w, img_h);

        // Viewer realized lazily; the EGL headless GL context is created eagerly
        // in the OSGViewRenderer constructor.
        renderer = std::make_unique<OSGViewRenderer>(*scene,
                                                     DefaultViewPoses(bowl_params.car_center,
                                                                      bowl_params.r_outter /
                                                                          kCalibratedAnchorRoutter),
                                                     output_width, output_height,
                                                     RenderMode::Offscreen,
                                                     bowl_params.car_center);

        num_cameras = N;
        configured = true;
    }

    std::map<std::string, cv::Mat> RenderFrame(
        const std::vector<cv::Mat>& images) const {
        if (!configured) {
            throw std::runtime_error("StitchEngine::RenderFrame called before Configure()");
        }
        if (images.size() != static_cast<size_t>(num_cameras)) {
            throw std::invalid_argument(
                "StitchEngine::RenderFrame: expected " + std::to_string(num_cameras) +
                " images, got " + std::to_string(images.size()));
        }

        scene->UploadCameraImages(images);
        return renderer->RenderAllViews();
    }

    bool IsConfigured() const { return configured; }

    int GetNumCameras() const { return num_cameras; }
};

StitchEngine::StitchEngine() : impl_(std::make_unique<Impl>()) {}

StitchEngine::~StitchEngine() = default;

void StitchEngine::Configure(const std::vector<CameraIntrinsics>& intrinsics,
                             const std::vector<CameraExtrinsics>& extrinsics,
                             const BowlParams& bowl_params,
                             int output_width, int output_height) {
    impl_->Configure(intrinsics, extrinsics, bowl_params,
                     output_width, output_height);
}

std::map<std::string, cv::Mat> StitchEngine::RenderFrame(
    const std::vector<cv::Mat>& images) const {
    return impl_->RenderFrame(images);
}

bool StitchEngine::IsConfigured() const { return impl_->IsConfigured(); }

int StitchEngine::GetNumCameras() const { return impl_->GetNumCameras(); }

} // namespace avm_stitch
