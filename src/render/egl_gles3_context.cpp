#include "render/egl_gles3_context.hpp"

namespace avm_worker { namespace avm_stitch {

EglGles3Context::EglGles3Context() = default;

EglGles3Context::~EglGles3Context() {
    Destroy();
}

bool EglGles3Context::Init() {
    // Idempotent: repeated Init() first tears down any previous EGL objects.
    Destroy();

    // EGL_EXT_platform_base entry point; the surfaceless platform is only
    // reachable through it (no X11 display involved). No fallback to
    // eglGetDisplay(EGL_DEFAULT_DISPLAY): on a desktop with X11 that call
    // silently creates an X11-connected display, defeating the "no X server"
    // headless claim.
    PFNEGLGETPLATFORMDISPLAYEXTPROC getPlatformDisplay =
        (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
    if (!getPlatformDisplay) return false;

    EGLDisplay dpy =
        getPlatformDisplay(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
    if (dpy == EGL_NO_DISPLAY) return false;
    display_ = dpy;

    if (!eglInitialize(dpy, nullptr, nullptr)) return false;
    initialized_ = true;

    // Key difference vs EGLHeadlessContext: bind the ES API (not desktop GL).
    if (!eglBindAPI(EGL_OPENGL_ES_API)) return false;

    const EGLint configAttribs[] = {
        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT_KHR,
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
        return false;
    }

    // Tiny pbuffer: the renderer draws into FBOs, not this surface.
    const EGLint pbAttribs[] = {EGL_WIDTH, 16, EGL_HEIGHT, 16, EGL_NONE};
    EGLSurface surf = eglCreatePbufferSurface(dpy, config, pbAttribs);
    if (surf == EGL_NO_SURFACE) return false;
    surface_ = surf;

    // GLES 3.0 context for GRAPHICS_API_OPENGL_ES3 (#version 300 es shaders).
    const EGLint ctxAttribs[] = {
        EGL_CONTEXT_CLIENT_VERSION, 3,
        EGL_NONE,
    };
    EGLContext ctx = eglCreateContext(dpy, config, EGL_NO_CONTEXT, ctxAttribs);
    if (ctx == EGL_NO_CONTEXT) return false;
    context_ = ctx;

    if (!eglMakeCurrent(dpy, surf, surf, ctx)) return false;

    valid_ = true;
    return true;
}

bool EglGles3Context::MakeCurrent() {
    return display_ != EGL_NO_DISPLAY &&
           eglMakeCurrent(display_, surface_, surface_, context_) == EGL_TRUE;
}

bool EglGles3Context::Release() {
    return display_ != EGL_NO_DISPLAY &&
           eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE,
                          EGL_NO_CONTEXT) == EGL_TRUE;
}

void EglGles3Context::Destroy() {
    if (display_ != EGL_NO_DISPLAY) {
        // Only tear down EGL objects on an initialized display. display_ is
        // set before eglInitialize succeeds; if Init failed at eglInitialize
        // (or earlier) there is nothing created yet, and calling eglTerminate
        // on an un-initialized display is undefined.
        if (initialized_) {
            eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE,
                           EGL_NO_CONTEXT);
            if (context_ != EGL_NO_CONTEXT) {
                eglDestroyContext(display_, context_);
                context_ = EGL_NO_CONTEXT;
            }
            if (surface_ != EGL_NO_SURFACE) {
                eglDestroySurface(display_, surface_);
                surface_ = EGL_NO_SURFACE;
            }
            eglTerminate(display_);
            initialized_ = false;
        }
        display_ = EGL_NO_DISPLAY;
    }
    valid_ = false;
}

}} // namespace avm_worker::avm_stitch
