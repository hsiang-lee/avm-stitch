// Tests for the headless EGL surfaceless GraphicsWindow (desktop GL 3.3
// compatibility profile, no window, no X server). These tests MUST pass with
// DISPLAY unset (`env -u DISPLAY ./build/test_egl_headless_context`): the EGL
// surfaceless platform ignores DISPLAY entirely.

// GL_GLEXT_PROTOTYPES must be defined before <GL/gl.h> is first included:
// the vcpkg osg GL2 build only defines it for GL3 builds, and the GLSL 330
// compile test needs the glCreateShader/glCompileShader/... prototypes.
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>

#include <gtest/gtest.h>

#include "avm_stitch/egl_headless_context.hpp"

#include <osg/GraphicsContext>

#include <string>

using namespace avm_worker::avm_stitch;

// The context must realize and report valid without any window/display.
TEST(EGLHeadlessContext, RealizesValidHeadless) {
    auto gc = CreateEGLHeadlessContext(16, 16);
    ASSERT_TRUE(gc.valid());
    EXPECT_TRUE(gc->realizeImplementation());
    EXPECT_TRUE(gc->valid());
    EXPECT_TRUE(gc->isRealizedImplementation());
    gc->closeImplementation();
    EXPECT_FALSE(gc->isRealizedImplementation());
}

// makeCurrent must succeed and expose a desktop-GL context (GL_VERSION
// non-null and not an ES string).
TEST(EGLHeadlessContext, MakeCurrentExposesDesktopGL) {
    auto gc = CreateEGLHeadlessContext(16, 16);
    ASSERT_TRUE(gc->realizeImplementation());
    EXPECT_TRUE(gc->makeCurrentImplementation());
    const char* ver = reinterpret_cast<const char*>(glGetString(GL_VERSION));
    ASSERT_NE(nullptr, ver);
    const std::string version(ver);
    // ES contexts report "OpenGL ES X.Y"; desktop GL reports "X.Y ...".
    EXPECT_EQ(std::string::npos, version.find("OpenGL ES"));
    // A compatibility profile names the profile; otherwise a desktop GL
    // version prefix (3.x/4.x) also proves this is not an ES context.
    EXPECT_TRUE(version.find("Compatibility") != std::string::npos ||
                version.compare(0, 2, "4.") == 0 ||
                version.compare(0, 2, "3.") == 0);
    gc->releaseContextImplementation();
    gc->closeImplementation();
}

// A GLSL 330 core shader must compile on the context (proves it is a real
// desktop GL 3.3+ context, not an ES-only context).
TEST(EGLHeadlessContext, GLSL330Compiles) {
    auto gc = CreateEGLHeadlessContext(16, 16);
    ASSERT_TRUE(gc->realizeImplementation());
    ASSERT_TRUE(gc->makeCurrentImplementation());
    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    ASSERT_NE(0u, vs);
    const char* src =
        "#version 330 core\n"
        "layout(location=0) in vec3 p;\n"
        "void main(){gl_Position=vec4(p,1.0);}\n";
    glShaderSource(vs, 1, &src, nullptr);
    glCompileShader(vs);
    GLint ok = 0;
    glGetShaderiv(vs, GL_COMPILE_STATUS, &ok);
    EXPECT_EQ(GL_TRUE, ok);
    glDeleteShader(vs);
    gc->closeImplementation();
}

// releaseContext then re-makeCurrent works (viewer frame() calls both).
TEST(EGLHeadlessContext, ReleaseThenRemakeCurrent) {
    auto gc = CreateEGLHeadlessContext(16, 16);
    ASSERT_TRUE(gc->realizeImplementation());
    EXPECT_TRUE(gc->makeCurrentImplementation());
    EXPECT_TRUE(gc->releaseContextImplementation());
    EXPECT_TRUE(gc->makeCurrentImplementation());
    gc->closeImplementation();
}
