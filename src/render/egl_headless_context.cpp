#include "render/egl_headless_context.hpp"

#include <osg/Notify>
#include <osg/State>

namespace avm_worker { namespace avm_stitch {

EGLHeadlessContext::EGLHeadlessContext(osg::GraphicsContext::Traits* traits) {
    _traits = traits;
    // Create the EGL display/surface/context eagerly (mirrors
    // GraphicsWindowX11::init()): osgViewer::Viewer::realize() collects the
    // camera's context via getContexts(onlyValid=true) BEFORE calling
    // realize(), so a context created only inside realizeImplementation() would
    // never be found and the viewer would fall back to an X11 window.
    InitEGL();
    if (valid_) {
        setState(new osg::State);
        getState()->setGraphicsContext(this);
        if (_traits.valid() && _traits->sharedContext.valid()) {
            getState()->setContextID(
                _traits->sharedContext->getState()->getContextID());
            incrementContextIDUsageCount(getState()->getContextID());
        } else {
            getState()->setContextID(osg::GraphicsContext::createNewContextID());
        }
    }
}

EGLHeadlessContext::~EGLHeadlessContext() {
    if (egl_display_ != EGL_NO_DISPLAY) closeImplementation();
}

bool EGLHeadlessContext::valid() const {
    return valid_;
}

bool EGLHeadlessContext::isRealizedImplementation() const {
    return realized_;
}

bool EGLHeadlessContext::InitEGL() {
    // EGL_EXT_platform_base entry point; the surfaceless platform is only
    // reachable through it (no X11 display involved).
    PFNEGLGETPLATFORMDISPLAYEXTPROC getPlatformDisplay =
        (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
    if (!getPlatformDisplay) {
        OSG_WARN << "EGLHeadlessContext: eglGetPlatformDisplayEXT unavailable"
                 << std::endl;
        valid_ = false;
        return false;
    }

    EGLDisplay dpy =
        getPlatformDisplay(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
    if (dpy == EGL_NO_DISPLAY) {
        // No fallback to eglGetDisplay(EGL_DEFAULT_DISPLAY): on a desktop with
        // X11 that call silently creates an X11-connected display, defeating
        // the "no X server" headless claim and giving false confidence before
        // docker deployment.
        OSG_WARN << "EGLHeadlessContext: EGL_PLATFORM_SURFACELESS_MESA unavailable (need Mesa EGL surfaceless platform)"
                 << std::endl;
        valid_ = false;
        return false;
    }
    egl_display_ = dpy;

    EGLint major = 0, minor = 0;
    if (!eglInitialize(dpy, &major, &minor)) {
        OSG_WARN << "EGLHeadlessContext: eglInitialize failed" << std::endl;
        valid_ = false;
        return false;
    }
    egl_initialized_ = true;
    if (!eglBindAPI(EGL_OPENGL_API)) {
        OSG_WARN << "EGLHeadlessContext: eglBindAPI(EGL_OPENGL_API) failed"
                 << std::endl;
        valid_ = false;
        return false;
    }

    const EGLint configAttribs[] = {
        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
        EGL_RED_SIZE, 8,
        EGL_GREEN_SIZE, 8,
        EGL_BLUE_SIZE, 8,
        EGL_ALPHA_SIZE, 8,
        EGL_DEPTH_SIZE, 24,
        EGL_NONE,
    };
    EGLConfig config = nullptr;
    EGLint numConfigs = 0;
    if (!eglChooseConfig(dpy, configAttribs, &config, 1, &numConfigs) ||
        numConfigs < 1) {
        OSG_WARN << "EGLHeadlessContext: eglChooseConfig failed" << std::endl;
        valid_ = false;
        return false;
    }

    // Tiny pbuffer: the stitch renderer renders into FBOs, not this surface.
    const EGLint pbAttribs[] = {EGL_WIDTH, 16, EGL_HEIGHT, 16, EGL_NONE};
    EGLSurface surf = eglCreatePbufferSurface(dpy, config, pbAttribs);
    if (surf == EGL_NO_SURFACE) {
        OSG_WARN << "EGLHeadlessContext: eglCreatePbufferSurface failed"
                 << std::endl;
        valid_ = false;
        return false;
    }
    egl_surface_ = surf;

    // Desktop GL 3.3 compatibility profile: the osg GL2 build calls legacy
    // APIs (glEnableClientState, glBegin/glEnd, ...) and needs VAO-free paths,
    // so a core profile would break it.
    const EGLint ctxAttribs[] = {
        EGL_CONTEXT_MAJOR_VERSION_KHR, 3,
        EGL_CONTEXT_MINOR_VERSION_KHR, 3,
        EGL_CONTEXT_OPENGL_PROFILE_MASK_KHR,
        EGL_CONTEXT_OPENGL_COMPATIBILITY_PROFILE_BIT_KHR,
        EGL_NONE,
    };
    EGLContext ctx = eglCreateContext(dpy, config, EGL_NO_CONTEXT, ctxAttribs);
    if (ctx == EGL_NO_CONTEXT) {
        OSG_WARN << "EGLHeadlessContext: eglCreateContext failed" << std::endl;
        valid_ = false;
        return false;
    }
    egl_context_ = ctx;

    if (!eglMakeCurrent(dpy, surf, surf, ctx)) {
        OSG_WARN << "EGLHeadlessContext: eglMakeCurrent failed" << std::endl;
        valid_ = false;
        return false;
    }

    valid_ = true;
    return true;
}

bool EGLHeadlessContext::realizeImplementation() {
    if (realized_) return true;
    // The EGL context is created in the constructor (like GraphicsWindowX11),
    // so realize only marks the context ready to render.
    realized_ = valid_;
    return realized_;
}

void EGLHeadlessContext::closeImplementation() {
    if (egl_display_ != EGL_NO_DISPLAY) {
        // Only tear down EGL objects on an initialized display. egl_display_ is
        // set before eglInitialize succeeds; if InitEGL failed at eglInitialize
        // (or earlier) there is nothing created yet, and calling eglTerminate on
        // an un-initialized display is undefined.
        if (egl_initialized_) {
            eglMakeCurrent(egl_display_, EGL_NO_SURFACE, EGL_NO_SURFACE,
                           EGL_NO_CONTEXT);
            if (egl_context_ != EGL_NO_CONTEXT) {
                eglDestroyContext(egl_display_, egl_context_);
                egl_context_ = EGL_NO_CONTEXT;
            }
            if (egl_surface_ != EGL_NO_SURFACE) {
                eglDestroySurface(egl_display_, egl_surface_);
                egl_surface_ = EGL_NO_SURFACE;
            }
            eglTerminate(egl_display_);
            egl_initialized_ = false;
        }
        egl_display_ = EGL_NO_DISPLAY;
    }
    realized_ = false;
    valid_ = false;
}

bool EGLHeadlessContext::makeCurrentImplementation() {
    return egl_display_ != EGL_NO_DISPLAY &&
           eglMakeCurrent(egl_display_, egl_surface_, egl_surface_,
                          egl_context_) == EGL_TRUE;
}

bool EGLHeadlessContext::makeContextCurrentImplementation(
    osg::GraphicsContext* readContext) {
    // Single-context design: read and draw are the same EGL context.
    // Not needed for the FBO-based single-threaded renderer.
    (void)readContext;
    return makeCurrentImplementation();
}

bool EGLHeadlessContext::releaseContextImplementation() {
    return egl_display_ != EGL_NO_DISPLAY &&
           eglMakeCurrent(egl_display_, EGL_NO_SURFACE, EGL_NO_SURFACE,
                          EGL_NO_CONTEXT) == EGL_TRUE;
}

void EGLHeadlessContext::swapBuffersImplementation() {
    // All rendering is into FBOs; the pbuffer only owns the GL context.
    // Nothing to present. (eglSwapBuffers on the dummy pbuffer is wasted work.)
}

void EGLHeadlessContext::grabFocus() {}

void EGLHeadlessContext::grabFocusIfPointerInWindow() {}

void EGLHeadlessContext::raiseWindow() {}

void EGLHeadlessContext::requestWarpPointer(float /*x*/, float /*y*/) {}

osg::ref_ptr<EGLHeadlessContext> CreateEGLHeadlessContext(int width, int height) {
    osg::ref_ptr<osg::GraphicsContext::Traits> traits =
        new osg::GraphicsContext::Traits;
    traits->x = 0;
    traits->y = 0;
    traits->width = width;
    traits->height = height;
    traits->windowName = "avm-stitch-egl";
    traits->red = 8;
    traits->green = 8;
    traits->blue = 8;
    traits->alpha = 8;
    traits->depth = 24;
    traits->pbuffer = true;
    traits->doubleBuffer = true;
    traits->vsync = false;
    traits->glContextVersion = "3.3";
    // glContextProfileMask is interpreted by X11/GLX windowing, not by our EGL
    // context; the compatibility profile is set explicitly in the EGL context
    // attributes inside InitEGL(). Leave it 0.
    traits->glContextProfileMask = 0;
    return new EGLHeadlessContext(traits.get());
}

}} // namespace avm_worker::avm_stitch
