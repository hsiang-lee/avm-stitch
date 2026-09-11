#pragma once

// Shared GL test fixture, moved verbatim from tests/core/test_stitch_engine.cpp.
// Skips the whole suite when no OpenGL context is available.

#include "render/egl_headless_context.hpp"

#include <gtest/gtest.h>

#include <GL/gl.h>

#include <cstdlib>

namespace avm_stitch::test {

// ---------------------------------------------------------------------------
// GL fixture: skip the whole suite when no OpenGL context is available.
// ---------------------------------------------------------------------------
class OSGGLTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!IsGLContextAvailable()) {
            GTEST_SKIP() << "OpenGL context not available (set AVM_STITCH_TEST_GL=1 to force)";
        }
    }

    static bool IsGLContextAvailable() {
        static const bool available = DetectGLContext();
        return available;
    }

    static bool DetectGLContext() {
        const char* env = std::getenv("AVM_STITCH_TEST_GL");
        if (env) return std::atoi(env) == 1;
        // Probe: a headless EGL surfaceless context (no X server, no DISPLAY)
        // whose GL_VERSION must be queryable once current.
        try {
            auto gc = CreateEGLHeadlessContext(1, 1);
            if (!gc->realizeImplementation()) return false;
            bool ok = gc->valid() && gc->makeCurrentImplementation();
            if (ok) {
                const char* ver =
                    reinterpret_cast<const char*>(glGetString(GL_VERSION));
                ok = ver != nullptr;
            }
            gc->closeImplementation();
            return ok;
        } catch (...) {
            return false;
        }
    }
};

} // namespace avm_stitch::test
