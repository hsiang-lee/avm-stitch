// GLES3 gl3.h ships prototypes for all core functions (unlike the vcpkg osg
// GL2 <GL/gl.h>, which only defines them for GL3 builds), so no
// GL_GLEXT_PROTOTYPES define is needed.
#include "render/fbo_manager.hpp"

namespace avm_worker { namespace avm_stitch {

namespace {
constexpr int kRgbaChannels = 4; // bytes per pixel for GL_RGBA
constexpr int kRgbChannels = 3;  // bytes per pixel for GL_RGB
} // namespace

FboManager::FboManager() = default;

FboManager::~FboManager() {
    Destroy();
}

FboManager::FboManager(FboManager&& other) noexcept
    : fbo_id_(other.fbo_id_)
    , color_tex_id_(other.color_tex_id_)
    , depth_rbo_id_(other.depth_rbo_id_)
    , width_(other.width_)
    , height_(other.height_)
    , initialized_(other.initialized_) {
    ResetSource(other);
}

FboManager& FboManager::operator=(FboManager&& other) noexcept {
    if (this != &other) {
        Destroy();
        fbo_id_ = other.fbo_id_;
        color_tex_id_ = other.color_tex_id_;
        depth_rbo_id_ = other.depth_rbo_id_;
        width_ = other.width_;
        height_ = other.height_;
        initialized_ = other.initialized_;
        ResetSource(other);
    }
    return *this;
}

bool FboManager::Init(int width, int height) {
    // If already initialized, destroy first
    Destroy();

    width_ = width;
    height_ = height;

    if (!CreateColorTexture(width, height)) {
        return false;
    }
    if (!CreateDepthBuffer(width, height)) {
        return false;
    }
    if (!CreateFbo()) {
        return false;
    }

    initialized_ = true;
    return true;
}

bool FboManager::CreateColorTexture(int width, int height) {
    glGenTextures(1, &color_tex_id_);
    glBindTexture(GL_TEXTURE_2D, color_tex_id_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
    return true;
}

bool FboManager::CreateDepthBuffer(int width, int height) {
    glGenRenderbuffers(1, &depth_rbo_id_);
    glBindRenderbuffer(GL_RENDERBUFFER, depth_rbo_id_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    return true;
}

bool FboManager::CreateFbo() {
    glGenFramebuffers(1, &fbo_id_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, color_tex_id_, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                              GL_RENDERBUFFER, depth_rbo_id_);

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    if (status != GL_FRAMEBUFFER_COMPLETE) {
        // Cleanup on failure and report via return value
        Destroy();
        return false;
    }
    return true;
}

void FboManager::Bind() {
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
}

void FboManager::Unbind() {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

std::vector<std::uint8_t> FboManager::GlReadPixels(int width, int height,
                                                   GLenum format) {
    const int channels =
        (format == GL_RGB) ? kRgbChannels : kRgbaChannels;
    std::vector<std::uint8_t> pixels(static_cast<size_t>(width) *
                                     static_cast<size_t>(height) * channels);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
    glReadPixels(0, 0, width, height, format, GL_UNSIGNED_BYTE, pixels.data());
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return pixels;
}

void FboManager::Destroy() {
    glDeleteFramebuffers(1, &fbo_id_);
    glDeleteTextures(1, &color_tex_id_);
    glDeleteRenderbuffers(1, &depth_rbo_id_);
    fbo_id_ = 0;
    color_tex_id_ = 0;
    depth_rbo_id_ = 0;
    width_ = 0;
    height_ = 0;
    initialized_ = false;
}

void FboManager::ResetSource(FboManager& other) noexcept {
    other.fbo_id_ = 0;
    other.color_tex_id_ = 0;
    other.depth_rbo_id_ = 0;
    other.width_ = 0;
    other.height_ = 0;
    other.initialized_ = false;
}

}} // namespace avm_worker::avm_stitch
