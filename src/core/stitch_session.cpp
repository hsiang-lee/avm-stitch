#include "core/stitch_session.hpp"

#include "render/egl_gles3_context.hpp"
#include "render/fbo_manager.hpp"

#include <stdexcept>
#include <utility>

namespace avm_worker { namespace avm_stitch {

class StitchSession::Impl {
public:
    explicit Impl(std::unique_ptr<RendererInterface> r)
        : renderer(std::move(r)) {}

    std::unique_ptr<RendererInterface> renderer;
    EglGles3Context egl;
    FboManager fbo;
    bool egl_current_ = false;
};

StitchSession::StitchSession(std::unique_ptr<RendererInterface> renderer)
    : impl_(std::make_unique<Impl>(std::move(renderer))) {}

StitchSession::~StitchSession() {
    EndSession();
}

void StitchSession::StartSession(const nlohmann::json&,
                                 const std::string&,
                                 const std::string&) {
    throw std::runtime_error("StartSession not implemented");
}

nlohmann::json StitchSession::FeedFrame(const nlohmann::json&) {
    throw std::runtime_error("FeedFrame not implemented");
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
