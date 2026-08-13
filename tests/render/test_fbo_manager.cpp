// Layer 2 component test: FboManager requires a live GL context.
// Uses the EGL surfaceless headless context; when EGL/GL is unavailable the
// tests are skipped (GTEST_SKIP) instead of crashing.

// GL_GLEXT_PROTOTYPES must be defined before <GL/gl.h> is first included:
// the vcpkg osg GL2 build only defines it for GL3 builds, and the FBO
// functions (glGenFramebuffers/glFramebufferTexture2D/...) need prototypes.
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>

#include <gtest/gtest.h>

#include "render/fbo_manager.hpp"
#include "render/egl_headless_context.hpp"

#include <cstdint>
#include <vector>

using namespace avm_worker::avm_stitch;

// ── GL-gated test fixture ──
// Creates a surfaceless EGL context before each test; skips if EGL/GL
// unavailable so the tests never crash on machines without a GPU.
class FboManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        egl_ctx_ = CreateEGLHeadlessContext(64, 64);
        if (!egl_ctx_ || !egl_ctx_->valid()) {
            GTEST_SKIP() << "EGL surfaceless context unavailable — skipping GL test";
        }
        egl_ctx_->realizeImplementation();
        if (!egl_ctx_->makeCurrentImplementation()) {
            GTEST_SKIP() << "EGL context makeCurrent failed — skipping GL test";
        }
    }

    void TearDown() override {
        if (egl_ctx_) {
            egl_ctx_->closeImplementation();
        }
    }

    osg::ref_ptr<EGLHeadlessContext> egl_ctx_;
};

TEST_F(FboManagerTest, InitReturnsValidFboId) {
    FboManager fbo;
    EXPECT_TRUE(fbo.Init(64, 64));
    EXPECT_NE(fbo.GetFboId(), 0u);
}

TEST_F(FboManagerTest, InitTwiceIsSafe) {
    FboManager fbo;
    EXPECT_TRUE(fbo.Init(64, 64));
    EXPECT_TRUE(fbo.Init(64, 64));
    // After re-init, FBO should still be valid
    EXPECT_NE(fbo.GetFboId(), 0u);
}

TEST_F(FboManagerTest, BindUnbindDoesNotCrash) {
    FboManager fbo;
    ASSERT_TRUE(fbo.Init(64, 64));
    ASSERT_NO_THROW(fbo.Bind());
    ASSERT_NO_THROW(fbo.Unbind());
}

TEST_F(FboManagerTest, GlReadPixelsReturnsCorrectSize) {
    FboManager fbo;
    ASSERT_TRUE(fbo.Init(2, 2));
    auto pixels = fbo.GlReadPixels(2, 2, GL_RGBA);
    EXPECT_EQ(pixels.size(), 16u); // 2*2*4
}

TEST_F(FboManagerTest, ClearThenGlReadPixelsPreservesColor) {
    FboManager fbo;
    ASSERT_TRUE(fbo.Init(4, 4));
    fbo.Bind();
    glClearColor(1.0f, 0.0f, 0.0f, 1.0f); // red
    glClear(GL_COLOR_BUFFER_BIT);
    fbo.Unbind();
    auto pixels = fbo.GlReadPixels(4, 4, GL_RGBA);
    ASSERT_EQ(pixels.size(), 64u); // 4*4*4
    // RGBA: first pixel should be red
    EXPECT_GE(pixels[0], 200); // R
    EXPECT_LE(pixels[1], 50);  // G
    EXPECT_LE(pixels[2], 50);  // B
    EXPECT_GE(pixels[3], 200); // A
}

TEST_F(FboManagerTest, DestroyAllowsReinit) {
    FboManager fbo;
    ASSERT_TRUE(fbo.Init(64, 64));
    fbo.Destroy();
    EXPECT_EQ(fbo.GetFboId(), 0u);
    // Re-init after destroy
    EXPECT_TRUE(fbo.Init(32, 32));
    EXPECT_NE(fbo.GetFboId(), 0u);
}

TEST_F(FboManagerTest, DoubleDestroyIsSafe) {
    FboManager fbo;
    ASSERT_TRUE(fbo.Init(64, 64));
    fbo.Destroy();
    ASSERT_NO_THROW(fbo.Destroy());
}
