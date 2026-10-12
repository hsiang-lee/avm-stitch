#include "core/stitch_engine.hpp"

#include "geometry/blend_weights.hpp"
#include "geometry/kb_camera_model.hpp"
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

    /// Validate multi-model Configure arguments (N range, null models, shared
    /// image size, bowl params). Throws std::invalid_argument on the first
    /// violation. Error texts mirror the legacy kb path (compat window).
    void ValidateConfigureArgs(const std::vector<Camera>& cameras,
                               const BowlParams& bowl_params) const {
        const int N = static_cast<int>(cameras.size());
        if (N == 0 || N > 8) {
            throw std::invalid_argument(
                "StitchEngine::Configure: N must be in [1, 8], got " + std::to_string(N));
        }
        for (int i = 0; i < N; ++i) {
            if (cameras[i].model == nullptr) {
                throw std::invalid_argument(
                    "StitchEngine::Configure: camera " + std::to_string(i) +
                    " has no model (Camera.model is null)");
            }
        }
        // The camera texture array is one shared 2D array sized from camera 0;
        // every camera image must match it, so reject heterogeneous image sizes.
        for (int i = 1; i < N; ++i) {
            if (cameras[i].model->width() != cameras[0].model->width() ||
                cameras[i].model->height() != cameras[0].model->height()) {
                throw std::invalid_argument(
                    "StitchEngine::Configure: all cameras must share the same image "
                    "size; camera 0 is " + std::to_string(cameras[0].model->width()) + "x" +
                    std::to_string(cameras[0].model->height()) + ", camera " + std::to_string(i) +
                    " is " + std::to_string(cameras[i].model->width()) + "x" +
                    std::to_string(cameras[i].model->height()));
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

    void Configure(const std::vector<Camera>& cameras,
                   const BowlParams& bowl_params,
                   int output_width, int output_height) {
        ValidateConfigureArgs(cameras, bowl_params);
        const int N = static_cast<int>(cameras.size());

        // Phase 1 pipeline: bowl model + bake UV/weights -> OSG scene graph.
        const BowlModel model(bowl_params, cameras);

        const int img_w = cameras[0].model->width();
        const int img_h = cameras[0].model->height();
        scene = std::make_unique<StitchScene>(model, img_w, img_h);

        // Viewer realized lazily; the EGL headless GL context is created eagerly
        // in the OSGViewRenderer constructor.
        renderer = std::make_unique<OSGViewRenderer>(
            *scene, bowl_params.r_outter,
            DefaultViewPoses(bowl_params.car_center,
                             bowl_params.r_outter / kCalibratedAnchorRoutter),
            output_width, output_height, RenderMode::Offscreen,
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
    // Legacy count check (was in the old ValidateConfigureArgs): the wrapper
    // indexes extrinsics[i] per intrinsics[i], so reject mismatches up front.
    if (intrinsics.size() != extrinsics.size()) {
        throw std::invalid_argument(
            "StitchEngine::Configure: intrinsics and extrinsics size mismatch");
    }
    // Legacy kb wrapper: wrap the intrinsics/extrinsics into kb Camera models
    // and delegate (sim 零感知 — the kb path is behaviourally unchanged).
    std::vector<Camera> cameras;
    cameras.reserve(intrinsics.size());
    for (size_t i = 0; i < intrinsics.size(); ++i) {
        Camera cam;
        cam.extrinsics = extrinsics[i];
        cam.model = std::make_unique<KbCameraModel>(intrinsics[i]);
        cameras.push_back(std::move(cam));
    }
    impl_->Configure(cameras, bowl_params, output_width, output_height);
}

void StitchEngine::Configure(const std::vector<Camera>& cameras,
                             const BowlParams& bowl_params,
                             int output_width, int output_height) {
    impl_->Configure(cameras, bowl_params, output_width, output_height);
}

std::map<std::string, cv::Mat> StitchEngine::RenderFrame(
    const std::vector<cv::Mat>& images) const {
    return impl_->RenderFrame(images);
}

bool StitchEngine::IsConfigured() const { return impl_->IsConfigured(); }

int StitchEngine::GetNumCameras() const { return impl_->GetNumCameras(); }

} // namespace avm_stitch
