
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>

#include <gtest/gtest.h>

#include "render/egl_headless_context.hpp"

#include "test_support/gl_fixture.hpp"

#include <osg/GraphicsContext>

#include <string>

using namespace avm_stitch;
using avm_stitch::test::OSGGLTest;

TEST_F(OSGGLTest, RealizesValidHeadless) {
    auto gc = CreateEGLHeadlessContext(16, 16);
    ASSERT_TRUE(gc.valid());
    EXPECT_TRUE(gc->realizeImplementation());
    EXPECT_TRUE(gc->valid());
    EXPECT_TRUE(gc->isRealizedImplementation());
    gc->closeImplementation();
    EXPECT_FALSE(gc->isRealizedImplementation());
}

TEST_F(OSGGLTest, MakeCurrentExposesDesktopGL) {
    auto gc = CreateEGLHeadlessContext(16, 16);
    ASSERT_TRUE(gc->realizeImplementation());
    EXPECT_TRUE(gc->makeCurrentImplementation());
    const char* ver = reinterpret_cast<const char*>(glGetString(GL_VERSION));
    ASSERT_NE(nullptr, ver);
    const std::string version(ver);
        EXPECT_EQ(std::string::npos, version.find("OpenGL ES"));
            EXPECT_TRUE(version.find("Compatibility") != std::string::npos ||
                version.compare(0, 2, "4.") == 0 ||
                version.compare(0, 2, "3.") == 0);
    gc->releaseContextImplementation();
    gc->closeImplementation();
}

TEST_F(OSGGLTest, GLSL330Compiles) {
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

TEST_F(OSGGLTest, ReleaseThenRemakeCurrent) {
    auto gc = CreateEGLHeadlessContext(16, 16);
    ASSERT_TRUE(gc->realizeImplementation());
    EXPECT_TRUE(gc->makeCurrentImplementation());
    EXPECT_TRUE(gc->releaseContextImplementation());
    EXPECT_TRUE(gc->makeCurrentImplementation());
    gc->closeImplementation();
}
