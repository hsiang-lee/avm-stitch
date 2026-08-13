// Task 6d: RealRenderer (avm-renderer wrapper) tests.
//
// Layer 1 (no GL, always runs): methods return false before Init, Destroy is
// idempotent. These guard against the renderer being used without a context.
//
// Layer 2 (GL-gated integration): requires a live EGL GLES3 context AND a
// real calibration fixture (dir containing model_*.bin) pointed at by the
// AVM_CALIB_FIXTURE env var. When either is missing the tests SKIP so the
// suite stays repeatable on machines without a GPU or fixture.

#include <gtest/gtest.h>

#include "core/real_renderer.hpp"
#include "render/egl_gles3_context.hpp"
#include "render/fbo_manager.hpp"

#include <cstdlib>
#include <cstdint>
#include <vector>

using namespace avm_worker::avm_stitch;

// ── Layer 1: no GL, always runs ──

TEST(RealRenderer, MethodsReturnFalseBeforeInit) {
    RealRenderer r;
    EXPECT_FALSE(r.UpdateView(22));
    EXPECT_FALSE(r.Render());
    r.Destroy();          // 未 Init 的 Destroy 安全（幂等）
    r.Destroy();
}

// ── Layer 2: GL-gated integration (requires EGL + real fixture) ──

// RED trigger: with a valid EGL+FBO+calib fixture, Init must return true.
// Against the cpp stub (returns false) this FAILS; the real implementation
// makes it PASS. Without the fixture it SKIPs.
TEST(RealRenderer, InitWithValidContextSucceeds) {
    EglGles3Context egl;
    if (!egl.Init() || !egl.MakeCurrent())
        GTEST_SKIP() << "EGL GLES3 unavailable";
    FboManager fbo;
    ASSERT_TRUE(fbo.Init(960, 640));
    RealRenderer r;
    const char* calib_env = std::getenv("AVM_CALIB_FIXTURE"); // 指向含 model_*.bin 的目录
    if (!calib_env || !*calib_env)
        GTEST_SKIP() << "AVM_CALIB_FIXTURE not set — skipping avm_renderer integration";
    EXPECT_TRUE(r.Init(calib_env, fbo.GetFboId(), 960, 640, 0));
    r.Destroy();
}

// GL-gated negative test (regression): valid EGL+FBO but nonexistent calib →
// avm_system_init fails → false.
TEST(RealRenderer, InitWithNonexistentCalibReturnsFalse) {
    EglGles3Context egl;
    if (!egl.Init() || !egl.MakeCurrent())
        GTEST_SKIP() << "EGL GLES3 unavailable";
    FboManager fbo;
    ASSERT_TRUE(fbo.Init(960, 640));
    RealRenderer r;
    EXPECT_FALSE(r.Init("/nonexistent/calib", fbo.GetFboId(), 960, 640, 0));
}
