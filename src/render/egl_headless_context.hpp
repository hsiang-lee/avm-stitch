#pragma once
// EGL surfaceless headless GraphicsWindow: creates a desktop-GL 3.3
// compatibility-profile context with no window and no X server. Lets the
// stitch renderer run in docker/CI without DISPLAY.

#include <osgViewer/GraphicsWindow>
#include <EGL/egl.h>
#include <EGL/eglext.h>

namespace avm_stitch {

/// Headless EGL surfaceless graphics window (desktop GL, no X11).
class EGLHeadlessContext : public osgViewer::GraphicsWindow {
public:
    explicit EGLHeadlessContext(osg::GraphicsContext::Traits* traits);
    // GraphicsContext overrides
    bool valid() const override;
    bool realizeImplementation() override;
    bool isRealizedImplementation() const override;
    void closeImplementation() override;
    bool makeCurrentImplementation() override;
    bool makeContextCurrentImplementation(osg::GraphicsContext* readContext) override;
    bool releaseContextImplementation() override;
    void swapBuffersImplementation() override;
    // GUIActionAdapter / GraphicsWindow
    void grabFocus() override;
    void grabFocusIfPointerInWindow() override;
    void raiseWindow() override;
    void requestWarpPointer(float x, float y) override;

protected:
    ~EGLHeadlessContext() override;

private:
    bool InitEGL();  // display + config + pbuffer + context

    EGLDisplay egl_display_ = EGL_NO_DISPLAY;
    EGLSurface egl_surface_ = EGL_NO_SURFACE;
    EGLContext egl_context_ = EGL_NO_CONTEXT;
    bool egl_initialized_ = false;
    bool realized_ = false;
    bool valid_ = false;
};

/// Create an EGLHeadlessContext from default traits (width x height, 3.3 compat).
osg::ref_ptr<EGLHeadlessContext> CreateEGLHeadlessContext(int width, int height);

} // namespace avm_stitch
