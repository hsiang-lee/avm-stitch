// GL_GLEXT_PROTOTYPES must be defined before <GL/gl.h> is first included
// (the vcpkg osg GL2 build only defines it for GL3 builds): the FBO functions
// (glGenFramebuffers/glFramebufferTexture2D/...) need their prototypes.
#define GL_GLEXT_PROTOTYPES
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
    , width_(other.width_)
    , height_(other.height_)
    , initialized_(other.initialized_) {
    other.fbo_id_ = 0;
    other.color_tex_id_ = 0;
    other.width_ = 0;
    other.height_ = 0;
    other.initialized_ = false;
}

FboManager& FboManager::operator=(FboManager&& other) noexcept {
    if (this != &other) {
        Destroy();
        fbo_id_ = other.fbo_id_;
        color_tex_id_ = other.color_tex_id_;
        width_ = other.width_;
        height_ = other.height_;
        initialized_ = other.initialized_;
        other.fbo_id_ = 0;
        other.color_tex_id_ = 0;
        other.width_ = 0;
        other.height_ = 0;
        other.initialized_ = false;
    }
    return *this;
}

bool FboManager::Init(int width, int height) {
    // If already initialized, destroy first
    Destroy();

    width_ = width;
    height_ = height;

    // Create texture as color attachment
    glGenTextures(1, &color_tex_id_);
    glBindTexture(GL_TEXTURE_2D, color_tex_id_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    // Create FBO
    glGenFramebuffers(1, &fbo_id_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, color_tex_id_, 0);

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    if (status != GL_FRAMEBUFFER_COMPLETE) {
        // Cleanup on failure and report via return value
        Destroy();
        return false;
    }

    initialized_ = true;
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
    fbo_id_ = 0;
    color_tex_id_ = 0;
    width_ = 0;
    height_ = 0;
    initialized_ = false;
}

}} // namespace avm_worker::avm_stitch
