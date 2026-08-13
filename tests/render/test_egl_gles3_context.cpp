// Tests for the standalone headless EGL surfaceless GLES3 context (no OSG, no
// window, no X server). This context serves FboManager / RealRenderer /
// RendererBackend which compile against GRAPHICS_API_OPENGL_ES3 (#version 300
// es shaders), so it must expose an OpenGL ES 3 API, unlike the OSG-specific
// EGLHeadlessContext (desktop GL 3.3 compatibility).
//
// GL-gated: if EGL surfaceless + ES3 config is unavailable (e.g. docker
// without GPU), tests SKIP rather than fail.

#define GL_GLEXT_PROTOTYPES
#include <GLES3/gl3.h>

#include <gtest/gtest.h>

#include "render/egl_gles3_context.hpp"

#include <string>

using namespace avm_worker::avm_stitch;

class EglGles3ContextTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!ctx_.Init() || !ctx_.valid())
            GTEST_SKIP() << "EGL GLES3 surfaceless context unavailable";
    }
    EglGles3Context ctx_;
};

TEST_F(EglGles3ContextTest, InitReturnsValidContext) {
    EXPECT_TRUE(ctx_.valid());
}

TEST_F(EglGles3ContextTest, ReportsOpenGLES3Version) {
    ASSERT_TRUE(ctx_.MakeCurrent());
    const char* ver = reinterpret_cast<const char*>(glGetString(GL_VERSION));
    ASSERT_NE(ver, nullptr);
    EXPECT_NE(std::string(ver).find("OpenGL ES 3"), std::string::npos);
}
