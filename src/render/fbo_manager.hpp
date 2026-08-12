#pragma once

#include <GL/gl.h>

#include <cstdint>
#include <vector>

namespace avm_worker { namespace avm_stitch {

/// Manages an OpenGL Framebuffer Object for off-screen rendering.
/// Must be used within an active GL context (e.g. the EGL headless context).
class FboManager {
public:
    FboManager();
    ~FboManager();

    // Non-copyable, movable
    FboManager(const FboManager&) = delete;
    FboManager& operator=(const FboManager&) = delete;
    FboManager(FboManager&& other) noexcept;
    FboManager& operator=(FboManager&& other) noexcept;

    /// Create the FBO with an RGBA8 color attachment.
    /// @param width   FBO width in pixels
    /// @param height  FBO height in pixels
    /// @return true on success; false if the FBO is incomplete
    bool Init(int width, int height);

    /// Return the OpenGL FBO ID (0 if not initialized).
    GLuint GetFboId() const { return fbo_id_; }

    /// Bind the FBO as the current render target.
    void Bind();

    /// Unbind (restore default framebuffer).
    void Unbind();

    /// Read pixels from the FBO into an RGBA8888 (or RGB) byte buffer.
    /// @param width   must match FBO width
    /// @param height  must match FBO height
    /// @param format  pixel format: GL_RGBA (4 bytes/px) or GL_RGB (3 bytes/px)
    /// @return pixel buffer, size = width * height * channels
    std::vector<std::uint8_t> GlReadPixels(int width, int height, GLenum format);

    /// Release GL resources. Safe to call multiple times.
    void Destroy();

private:
    GLuint fbo_id_ = 0;
    GLuint color_tex_id_ = 0;
    int width_ = 0;
    int height_ = 0;
    bool initialized_ = false;
};

}} // namespace avm_worker::avm_stitch
