#include "core/stitch_engine.hpp"

#include "bowl_model/blend_weights.hpp"
#include "render/stitch_scene.hpp"
#include "render/osg_view_renderer.hpp"

#include <stdexcept>
#include <string>
#include <utility>

namespace avm_stitch {

namespace {

void ValidateConfigureArgs(const std::vector<Camera>& cameras,
                           const BowlParams& bowl_params) {
    const int N = static_cast<int>(cameras.size());
    if (N == 0 || N > 8) {
        throw std::invalid_argument(
            "StitchEngine::Configure: N must be in [1, 8], got " + std::to_string(N));
    }
    for (int i = 0; i < N; ++i) {
        if (!cameras[i].Valid()) {
            throw std::invalid_argument(
                "StitchEngine::Configure: camera " + std::to_string(i) +
                " has no model (Camera.model is null)");
        }
    }
            for (int i = 1; i < N; ++i) {
        if (cameras[i].Width() != cameras[0].Width() ||
            cameras[i].Height() != cameras[0].Height()) {
            throw std::invalid_argument(
                "StitchEngine::Configure: all cameras must share the same image "
                "size; camera 0 is " + std::to_string(cameras[0].Width()) + "x" +
                std::to_string(cameras[0].Height()) + ", camera " + std::to_string(i) +
                " is " + std::to_string(cameras[i].Width()) + "x" +
                std::to_string(cameras[i].Height()));
        }
    }
            if (bowl_params.step() <= 0.0) {
        throw std::invalid_argument(
            "StitchEngine::Configure: bowl_params.step must be > 0, got " +
            std::to_string(bowl_params.step()));
    }
    if (bowl_params.r_outter() <= 0.0) {
        throw std::invalid_argument(
            "StitchEngine::Configure: bowl_params.r_outter must be > 0, got " +
            std::to_string(bowl_params.r_outter()));
    }
    if (bowl_params.k_slop() < 0.0) {
        throw std::invalid_argument(
            "StitchEngine::Configure: bowl_params.k_slop must be >= 0, got " +
            std::to_string(bowl_params.k_slop()));
    }
    if (bowl_params.z_exp() < 0.0) {
        throw std::invalid_argument(
            "StitchEngine::Configure: bowl_params.z_exp must be >= 0, got " +
            std::to_string(bowl_params.z_exp()));
    }
}

}
StitchEngine::StitchEngine() = default;

StitchEngine::~StitchEngine() = default;

void StitchEngine::Configure(std::vector<std::unique_ptr<CameraModel>> models,
                             const std::vector<CameraExtrinsics>& extrinsics,
                             const BowlParams& bowl_params,
                             int output_width, int output_height) {
            if (models.size() != extrinsics.size()) {
        throw std::invalid_argument(
            "StitchEngine::Configure: intrinsics and extrinsics size mismatch");
    }
        // Camera value object and run the full pipeline inline.
    std::vector<Camera> cameras;
    cameras.reserve(models.size());
    for (size_t i = 0; i < models.size(); ++i) {
        cameras.emplace_back(extrinsics[i], std::move(models[i]));
    }
    ValidateConfigureArgs(cameras, bowl_params);

    const BowlModel model(bowl_params, cameras);

    const int img_w = cameras[0].Width();
    const int img_h = cameras[0].Height();
    scene_ = std::make_unique<StitchScene>(model, img_w, img_h);

    renderer_ = std::make_unique<OSGViewRenderer>(
        *scene_, bowl_params.r_outter(),
        DefaultViewPoses(bowl_params.car_center(),
                         bowl_params.r_outter() / kCalibratedAnchorRoutter),
        output_width, output_height, RenderMode::Offscreen,
        bowl_params.car_center());

    configured_ = true;
}

std::map<std::string, cv::Mat> StitchEngine::RenderFrame(
    const std::vector<cv::Mat>& images) const {
    if (!configured_) {
        throw std::runtime_error("StitchEngine::RenderFrame called before Configure()");
    }
            const int expected = scene_->num_cameras();
    if (images.size() != static_cast<size_t>(expected)) {
        throw std::invalid_argument(
            "StitchEngine::RenderFrame: expected " + std::to_string(expected) +
            " images, got " + std::to_string(images.size()));
    }

    scene_->UploadCameraImages(images);
    return renderer_->RenderAllViews();
}

bool StitchEngine::IsConfigured() const { return configured_; }

int StitchEngine::GetNumCameras() const {
    return scene_ ? scene_->num_cameras() : 0;   }

}
