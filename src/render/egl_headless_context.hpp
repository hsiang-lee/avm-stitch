#pragma once

#include <osgViewer/GraphicsWindow>
#include <EGL/egl.h>
#include <EGL/eglext.h>

namespace avm_stitch {

class EGLHeadlessContext : public osgViewer::GraphicsWindow {
public:
    explicit EGLHeadlessContext(osg::GraphicsContext::Traits* traits);
        bool valid() const override;
    bool realizeImplementation() override;
    bool isRealizedImplementation() const override;
    void closeImplementation() override;
    bool makeCurrentImplementation() override;
    bool makeContextCurrentImplementation(osg::GraphicsContext* readContext) override;
    bool releaseContextImplementation() override;
    void swapBuffersImplementation() override;
        void grabFocus() override;
    void grabFocusIfPointerInWindow() override;
    void raiseWindow() override;
    void requestWarpPointer(float x, float y) override;

protected:
    ~EGLHeadlessContext() override;

private:
    bool InitEGL();
    EGLDisplay egl_display_ = EGL_NO_DISPLAY;
    EGLSurface egl_surface_ = EGL_NO_SURFACE;
    EGLContext egl_context_ = EGL_NO_CONTEXT;
    bool egl_initialized_ = false;
    bool realized_ = false;
    bool valid_ = false;
};

osg::ref_ptr<EGLHeadlessContext> CreateEGLHeadlessContext(int width, int height);

}
