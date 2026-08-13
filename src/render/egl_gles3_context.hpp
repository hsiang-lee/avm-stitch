#pragma once
#include <EGL/egl.h>
#include <EGL/eglext.h>

namespace avm_worker { namespace avm_stitch {

// Standalone headless EGL surfaceless GLES3 context (no OSG, no window, no X
// server). Unlike EGLHeadlessContext (desktop GL 3.3 compatibility for the
// OSG backend), this context binds EGL_OPENGL_ES_API and creates a
// GLES 3.0 context, matching avm_renderer's GRAPHICS_API_OPENGL_ES3 (#version
// 300 es shaders). FboManager / RealRenderer / RendererBackend use it.
class EglGles3Context {
public:
    EglGles3Context();
    ~EglGles3Context();
    EglGles3Context(const EglGles3Context&) = delete;
    EglGles3Context& operator=(const EglGles3Context&) = delete;
    bool Init();           // display + config + pbuffer + GLES3 context + makeCurrent
    bool MakeCurrent();    // eglMakeCurrent(surface, surface, context)
    bool Release();        // eglMakeCurrent(NO_SURFACE, NO_SURFACE, NO_CONTEXT)
    bool valid() const { return valid_; }
    void Destroy();
private:
    EGLDisplay display_ = EGL_NO_DISPLAY;
    EGLSurface surface_ = EGL_NO_SURFACE;
    EGLContext context_ = EGL_NO_CONTEXT;
    bool initialized_ = false;
    bool valid_ = false;
};

}} // namespace avm_worker::avm_stitch
