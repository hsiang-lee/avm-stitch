#pragma once

#include <GLES3/gl3.h>

#include <cstdint>
#include <vector>

namespace avm_worker { namespace avm_stitch {

/// Manages an OpenGL Framebuffer Object for off-screen rendering.
/// Uses the GLES3 API (GRAPHICS_API_OPENGL_ES3 in avm_renderer): GL objects
/// do not cross contexts, so FboManager must share the same GLES3 context as
/// the renderer (EglGles3Context), not the OSG desktop-GL context.
class FboManager {
public:
    FboManager();
    ~FboManager();

    // Non-copyable, movable
    FboManager(const FboManager&) = delete;
    FboManager& operator=(const FboManager&) = delete;
    FboManager(FboManager&& other) noexcept;
    FboManager& operator=(FboManager&& other) noexcept;

    /// Create the FBO with an RGBA8 color attachment and a depth renderbuffer.
    /// @param width   FBO width in pixels
    /// @param height  FBO height in pixels
    /// @return true on success; false if the FBO is incomplete
    bool Init(int width, int height);

    /// Return the OpenGL FBO ID (0 if not initialized).
    GLuint GetFboId() const { return fbo_id_; }

    /// Return the color attachment texture ID (0 if not initialized).
    GLuint GetColorId() const { return color_tex_id_; }

    /// Return the depth attachment renderbuffer ID (0 if not initialized).
    GLuint GetDepthId() const { return depth_rbo_id_; }

    /// Bind the FBO as the current render target.
    void Bind();

    /// Unbind (restore default framebuffer).
    void Unbind();

    /// Read pixels from the FBO into an RGBA8888 (or RGB) byte buffer.
    /// @param width   must match FBO width
    /// @param height  must match FBO height
    /// @param format  pixel format: GL_RGBA (4 bytes/px) or GL_RGB (3 bytes/px).
    ///                Note: GLES3 glReadPixels only guarantees GL_RGBA; the
    ///                GL_RGB branch is kept for desktop-GL callers and may be
    ///                unsupported on strict GLES3 implementations.
    /// @return pixel buffer, size = width * height * channels
    std::vector<std::uint8_t> GlReadPixels(int width, int height, GLenum format);

    /// Release GL resources. Safe to call multiple times.
    void Destroy();

private:
    /// Create the RGBA color attachment texture (no FBO yet).
    /// @return true if the texture was created
    bool CreateColorTexture(int width, int height);

    /// Create the FBO with the color texture and depth renderbuffer attached
    /// and verify completeness.
    /// @return true if the FBO is complete
    bool CreateFbo();

    /// Create the depth renderbuffer attachment (GL_DEPTH_COMPONENT24).
    /// @return true on success
    bool CreateDepthBuffer(int width, int height);

    /// Null out another instance's handles without touching GL resources
    /// (used by move operations so the moved-from object is empty).
    static void ResetSource(FboManager& other) noexcept;

    GLuint fbo_id_ = 0;
    GLuint color_tex_id_ = 0;
    GLuint depth_rbo_id_ = 0;
    int width_ = 0;
    int height_ = 0;
    bool initialized_ = false;
};

}} // namespace avm_worker::avm_stitch
