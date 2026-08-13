// RealRenderer: real avm-renderer wrapper (Task 6d-B).
//
// The avm_renderer types stay in the global namespace (::Render,
// ::AVM_INIT_PARAMS, ...) and are only referenced from the
// AVMRENDERER_AVAILABLE branch, keeping the header free of the dependency.
// Init() does NOT create an EGL context — the caller must have a current
// GLES3 context (the EGL context is owned and managed by RendererBackend).
#include "core/real_renderer.hpp"

#ifdef AVMRENDERER_AVAILABLE
#include <EGL/egl.h>
#include <GLES3/gl3.h>
// 必须先注册 fmt formatter（vector/raylib 类型）再 include Render.h：
// CarRender.hpp 在 AvmRender.hpp 中先于 spdlog_utils.hpp 被包含，直接
// include Render.h 会在 spdlog/fmt 编译期检查时因找不到
// std::vector<bool>/std::vector<T> 的 formatter 而失败。
#include <avm_renderer/utils/spdlog_utils.hpp>
#include <avm_renderer/Render.h>

#include <filesystem>
#include <string>
#include <vector>

namespace avm_worker { namespace avm_stitch {

struct RealRenderer::Impl {
    std::unique_ptr<::Render> render;
    int output_w = 0, output_h = 0;
    bool initialized = false;
};

RealRenderer::RealRenderer() : impl_(std::make_unique<Impl>()) {}
RealRenderer::~RealRenderer() = default;

bool RealRenderer::Init(const std::string& calib_dir, unsigned int fbo,
                        int w, int h, int flags) {
    (void)flags;
    Destroy();
    // 前置条件: 调用方已 MakeCurrent（EGL 上下文由 RendererBackend 管理）
    if (w <= 0 || h <= 0) return false;

    // avm_renderer 对缺失标定文件会宽容降级（avm_system_init 仍返回成功，
    // 但会渲染空模型）。生产流程中 calib 由上游生成，缺失即上游 bug，
    // 这里快速失败，避免静默渲染垃圾结果。
    if (!std::filesystem::exists(calib_dir)) return false;

    // resources_dir 推导: calib_dir 的兄弟目录 resources
    std::string resources_dir =
        (std::filesystem::path(calib_dir).parent_path() / "resources").string();

    // 查询 FBO 已绑定的 color/depth attachment id（Task 6b 保证已 attach）
    GLint color_id = 0, depth_id = 0;
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
        GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &color_id);
    glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
        GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &depth_id);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    ::AVM_INIT_PARAMS p{};
    p.target_fps = 0;
    p.camera_image_width = 0;    // load_configs() 从 models_config.json 覆盖
    p.camera_image_height = 0;
    p.output_width = (unsigned)w;
    p.output_height = (unsigned)h;
    p.aerial_view_percent = 0.375f;
    p.output_framebuffer_id = fbo;
    p.output_color_buffer_id = (unsigned)color_id;
    p.output_depth_buffer_id = (unsigned)depth_id;
    p.resources_dir_path = resources_dir;
    p.calib_dir_path = calib_dir;

    impl_->render = std::make_unique<::Render>(w, h);   // 构造需 current GL 上下文
    if (impl_->render->avm_system_init(p) != ::AvmResultCode::AVM_RES_SUCCESS)
        return false;
    impl_->output_w = w; impl_->output_h = h;
    impl_->initialized = true;
    return true;
}

bool RealRenderer::UpdateFrame(const std::vector<const std::uint8_t*>& buf,
                               int w, int h, int fmt) {
    if (!impl_->initialized) return false;
    ::AVM_INPUT_FRAME frame{};
    frame.image_data.image_width  = (unsigned)w;
    frame.image_data.image_height = (unsigned)h;
    frame.image_data.image_format = (::InputImageFormat)fmt;  // RGBA8888=0
    for (int i = 0; i < 4 && i < (int)buf.size(); ++i)
        frame.image_data.image_data[i] = const_cast<unsigned char*>(buf[i]);
    return impl_->render->avm_update_frame(frame) == ::AvmResultCode::AVM_RES_SUCCESS;
}

bool RealRenderer::UpdateView(int mode) {
    if (!impl_->initialized) return false;
    ::AVM_VIEW v{};
    v.render_mode = ::RenderMode::OPEN_AVM;
    v.avm_display_mode = (::AVMDisplayMode)mode;
    return impl_->render->avm_update_view(v) == ::AvmResultCode::AVM_RES_SUCCESS;
}

bool RealRenderer::Render() {
    if (!impl_->initialized) return false;
    return impl_->render->update() == ::AvmResultCode::AVM_RES_SUCCESS;
}

void RealRenderer::Destroy() {
    impl_->render.reset();          // 先析构 Render（其析构清理 GL 资源）
    impl_->initialized = false;     // 不销毁 EGL（不拥有）
}

}} // namespace avm_worker::avm_stitch

#else  // AVMRENDERER_AVAILABLE off: stub 全返回 false
namespace avm_worker { namespace avm_stitch {
struct RealRenderer::Impl {};
RealRenderer::RealRenderer() : impl_(std::make_unique<Impl>()) {}
RealRenderer::~RealRenderer() = default;
bool RealRenderer::Init(const std::string&, unsigned int, int, int, int) { return false; }
bool RealRenderer::UpdateFrame(const std::vector<const std::uint8_t*>&, int, int, int) { return false; }
bool RealRenderer::UpdateView(int) { return false; }
bool RealRenderer::Render() { return false; }
void RealRenderer::Destroy() {}
}} // namespace avm_worker::avm_stitch
#endif
