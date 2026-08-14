#include "core/stitch_session.hpp"

#include "common/image_converter.hpp"
#include "core/calib_files_generator.hpp"
#include "core/renderer_helpers.hpp"
#include "core/view_mapping.hpp"
#include "render/egl_gles3_context.hpp"
#include "render/fbo_manager.hpp"

#include <nlohmann/json.hpp>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace avm_worker { namespace avm_stitch {

class StitchSession::Impl {
public:
    explicit Impl(std::unique_ptr<RendererInterface> r)
        : renderer(std::move(r)) {}

    std::unique_ptr<RendererInterface> renderer;
    EglGles3Context egl;
    FboManager fbo;
    nlohmann::json calibration_;
    std::string task_base_dir_;
    bool egl_current_ = false;
};

StitchSession::StitchSession(std::unique_ptr<RendererInterface> renderer)
    : impl_(std::make_unique<Impl>(std::move(renderer))) {}

StitchSession::~StitchSession() {
    EndSession();
}

void StitchSession::StartSession(const nlohmann::json& calibration,
                                 const std::string& car_model_base64,
                                 const std::string& task_base_dir) {
    // car_model_base64 保留签名；当前 renderer 管线不消费。
    (void)car_model_base64;

    if (!impl_->renderer) {
        throw std::runtime_error("avm-renderer library not available");
    }

    std::filesystem::create_directories(task_base_dir + "/calib/data");
    std::filesystem::create_directories(task_base_dir + "/resources/config");

    const char* td = std::getenv("TEMPLATES_DIR");
    std::string templates = td ? td : "tools";
    CalibFilesGenerator(templates).Generate(calibration, task_base_dir);

    // model_creator 子进程 —— 仅此一次（会话初始化）。
    RunModelCreator(task_base_dir);

    CopyResources(task_base_dir);

    if (!impl_->egl.Init() || !impl_->egl.MakeCurrent()) {
        throw std::runtime_error("EGL GLES3 context init failed");
    }
    impl_->egl_current_ = true;

    if (!impl_->fbo.Init(kOutputWidth, kOutputHeight)) {
        throw std::runtime_error("FBO init failed");
    }

    if (!impl_->renderer->Init(task_base_dir + "/calib", impl_->fbo.GetFboId(),
                               kOutputWidth, kOutputHeight, 0)) {
        throw std::runtime_error("avm-renderer init failed");
    }

    impl_->calibration_ = calibration;
    impl_->task_base_dir_ = task_base_dir;
}

nlohmann::json StitchSession::FeedFrame(const nlohmann::json& images) {
    std::array<DecodedImage, 4> imgs;
    int iw = 0, ih = 0;
    auto buffers = DecodeRoleBuffers(images, impl_->calibration_, iw, ih, imgs);

    nlohmann::json images_obj = nlohmann::json::object();
    nlohmann::json view_errors = nlohmann::json::object();
    for (const char* view : kRenderViewNames) {
        if (!impl_->renderer->UpdateView(MapViewNameToDisplayMode(view))) {
            view_errors[view] = "UpdateView failed";
            continue;
        }
        if (!impl_->renderer->UpdateFrame(buffers, iw, ih, 0)) {
            view_errors[view] = "UpdateFrame failed";
            continue;
        }
        if (!impl_->renderer->Render()) {
            view_errors[view] = "Render failed";
            continue;
        }
        images_obj[view] = ReadbackViewAsBase64Png(
            impl_->fbo, kOutputWidth, kOutputHeight);
    }

    if (images_obj.empty()) {
        throw std::runtime_error("All views failed");
    }

    nlohmann::json result;
    result["images"] = images_obj;
    if (!view_errors.empty()) {
        result["view_errors"] = view_errors;
    }
    return result;
}

void StitchSession::EndSession() {
    if (impl_->renderer) {
        impl_->renderer->Destroy();
    }
    if (impl_->egl_current_) {
        impl_->fbo.Destroy();
        impl_->egl.Destroy();
        impl_->egl_current_ = false;
    }
}

}} // namespace avm_worker::avm_stitch
