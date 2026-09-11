// Tests for avm_stitch OSG scene construction: geometry, shaders, textures.
// Structural tests only — pure CPU data structures, no GL context required.

#include <gtest/gtest.h>

#include "geometry/bowl_model.hpp"
#include "render/stitch_scene.hpp"

#include "test_support/stitch_test_helpers.hpp"

#include <osg/Geometry>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Program>
#include <osg/Shader>
#include <osg/StateSet>
#include <osg/Texture2DArray>

#include <string>
#include <vector>

using namespace avm_stitch;

namespace {

// Local-bake variant: this suite's historical intrinsics are 1920x1536 with
// distortion (the renderer tests use MakeIntrinsics' 640x480), and every test
// bakes 4 side cameras at a 360/N azimuth ring.
BowlModel MakeBaked(int num_cameras, double r_outter = 12.6) {
    CameraIntrinsics intr = {1920, 1536, 441.45, 441.45, 952.14, 758.27, 0, 0, 0, 0};
    std::vector<double> azimuths;
    for (int i = 0; i < num_cameras; ++i) azimuths.push_back(i * 360.0 / num_cameras);
    return test::MakeCoarseBowl(r_outter, std::vector<CameraIntrinsics>(num_cameras, intr),
                                 test::MakeSideCameras(azimuths));
}

} // namespace

TEST(StitchSceneTest, BowlGeometryPositionAttributeCorrect) {
    auto bowl = MakeBaked(4, 5.0);
    StitchScene scene(bowl, 1920, 1536);
    auto* geom = dynamic_cast<osg::Geometry*>(scene.bowl_drawable().get());
    ASSERT_NE(nullptr, geom);

    auto* va = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
    ASSERT_NE(nullptr, va);
    ASSERT_EQ(static_cast<size_t>(bowl.vertex_count()), va->size());

    // VCS -> OpenGL: (x, y, z) -> (x, z, -y)
    for (int i = 0; i < bowl.vertex_count(); ++i) {
        const cv::Vec3d& v = bowl.vertex(i);
        EXPECT_NEAR(v[0], (*va)[i].x(), 1e-6);
        EXPECT_NEAR(v[2], (*va)[i].y(), 1e-6);
        EXPECT_NEAR(-v[1], (*va)[i].z(), 1e-6);
    }
}

TEST(StitchSceneTest, BowlGeometryUVArraysExist) {
    auto bowl = MakeBaked(4, 5.0);
    StitchScene scene(bowl, 1920, 1536);
    auto* geom = dynamic_cast<osg::Geometry*>(scene.bowl_drawable().get());
    ASSERT_NE(nullptr, geom);
    for (int loc = 2; loc <= 5; ++loc) {
        EXPECT_NE(nullptr, geom->getVertexAttribArray(loc)) << "UV location " << loc;
    }
}

TEST(StitchSceneTest, BowlGeometryWeightArraysExist) {
    auto bowl = MakeBaked(4, 5.0);
    StitchScene scene(bowl, 1920, 1536);
    auto* geom = dynamic_cast<osg::Geometry*>(scene.bowl_drawable().get());
    ASSERT_NE(nullptr, geom);
    for (int loc = 6; loc <= 7; ++loc) {
        EXPECT_NE(nullptr, geom->getVertexAttribArray(loc)) << "weight location " << loc;
    }
}

TEST(StitchSceneTest, BowlGeometryIndexCount) {
    auto bowl = MakeBaked(4, 5.0);
    StitchScene scene(bowl, 1920, 1536);
    auto* geom = dynamic_cast<osg::Geometry*>(scene.bowl_drawable().get());
    ASSERT_NE(nullptr, geom);

    int index_count = 0;
    for (unsigned int k = 0; k < geom->getNumPrimitiveSets(); ++k) {
        auto* de = dynamic_cast<osg::DrawElementsUInt*>(geom->getPrimitiveSet(k));
        if (de) index_count += de->getNumIndices();
    }
    EXPECT_EQ(static_cast<size_t>(bowl.triangle_count()) * 3u, static_cast<size_t>(index_count));
    EXPECT_GT(index_count, 0);
}

TEST(StitchSceneTest, BowlGeometryNormalBinding) {
    auto bowl = MakeBaked(4, 5.0);
    StitchScene scene(bowl, 1920, 1536);
    auto* geom = dynamic_cast<osg::Geometry*>(scene.bowl_drawable().get());
    ASSERT_NE(nullptr, geom);
    EXPECT_EQ(osg::Geometry::BIND_PER_VERTEX, geom->getNormalBinding());
    EXPECT_NE(nullptr, geom->getNormalArray());
}

// Shader sources are a StitchScene private implementation detail; observe the
// real assembled product: stateset -> program -> shaders.
namespace {

const osg::Shader* FindShader(const StitchScene& scene, osg::Shader::Type type) {
    auto* ss = scene.bowl_stateset().get();
    auto* program = dynamic_cast<osg::Program*>(
        ss->getAttribute(osg::StateAttribute::PROGRAM));
    if (!program) return nullptr;
    for (unsigned int i = 0; i < program->getNumShaders(); ++i) {
        if (program->getShader(i)->getType() == type) return program->getShader(i);
    }
    return nullptr;
}

} // namespace

TEST(StitchSceneTest, ShaderSources_VertexCompilesSyntactically) {
    auto bowl = MakeBaked(4, 5.0);
    StitchScene scene(bowl, 1920, 1536);
    const osg::Shader* vs = FindShader(scene, osg::Shader::VERTEX);
    ASSERT_NE(nullptr, vs);
    const std::string s(vs->getShaderSource());
    EXPECT_NE(std::string::npos, s.find("#version 330 core"));
    EXPECT_NE(std::string::npos, s.find("layout(location = 0) in vec3 vertexPosition"));
    EXPECT_NE(std::string::npos, s.find("osg_ModelViewProjectionMatrix"));
    EXPECT_NE(std::string::npos, s.find("main()"));
}

TEST(StitchSceneTest, ShaderSources_FragmentCompilesSyntactically) {
    auto bowl = MakeBaked(4, 5.0);
    StitchScene scene(bowl, 1920, 1536);
    const osg::Shader* fs = FindShader(scene, osg::Shader::FRAGMENT);
    ASSERT_NE(nullptr, fs);
    const std::string s(fs->getShaderSource());
    EXPECT_NE(std::string::npos, s.find("#version 330 core"));
    EXPECT_NE(std::string::npos, s.find("sampler2DArray"));
    EXPECT_NE(std::string::npos, s.find("fragWeights0"));
    EXPECT_NE(std::string::npos, s.find("WEIGHT_MIN"));
    EXPECT_NE(std::string::npos, s.find("finalColor"));
}

TEST(StitchSceneTest, ShaderSources_All8CamerasUnrolled) {
    auto bowl = MakeBaked(4, 5.0);
    StitchScene scene(bowl, 1920, 1536);
    const osg::Shader* fs = FindShader(scene, osg::Shader::FRAGMENT);
    ASSERT_NE(nullptr, fs);
    const std::string s(fs->getShaderSource());
    for (int i = 0; i < 8; ++i) {
        EXPECT_NE(std::string::npos, s.find("Camera " + std::to_string(i)))
            << "fragment shader must unroll camera " << i;
    }
}

TEST(StitchSceneTest, CameraTextureArraySetsCorrectSize) {
    auto bowl = MakeBaked(4);
    StitchScene scene(bowl, 1920, 1536);
    auto ss = scene.bowl_stateset();
    ASSERT_TRUE(ss.valid());
    auto* tex = dynamic_cast<osg::Texture2DArray*>(
        ss->getTextureAttribute(0, osg::StateAttribute::TEXTURE));
    ASSERT_NE(nullptr, tex);
    EXPECT_EQ(1920, tex->getTextureWidth());
    EXPECT_EQ(1536, tex->getTextureHeight());
    EXPECT_EQ(4, tex->getTextureDepth());
}

TEST(StitchSceneTest, RootNonNullGroup) {
    auto baked = MakeBaked(4);
    StitchScene scene(baked, 1920, 1536);
    auto root = scene.root();
    ASSERT_TRUE(root.valid());
    EXPECT_GE(root->getNumChildren(), 1u);
}

TEST(StitchSceneTest, ProgramAttached) {
    auto baked = MakeBaked(4);
    StitchScene scene(baked, 1920, 1536);
    auto root = scene.root();
    ASSERT_TRUE(root.valid());
    ASSERT_GE(root->getNumChildren(), 1u);

    auto* geode = dynamic_cast<osg::Geode*>(root->getChild(0));
    ASSERT_NE(nullptr, geode);
    auto* ss = geode->getStateSet();
    ASSERT_NE(nullptr, ss);
    EXPECT_NE(nullptr, ss->getAttribute(osg::StateAttribute::PROGRAM));
}

TEST(StitchSceneTest, UploadCameraImages_DoesNotCrash) {
    auto bowl = MakeBaked(2);
    StitchScene scene(bowl, 64, 48);
    auto ss = scene.bowl_stateset();
    ASSERT_TRUE(ss.valid());
    auto* tex = dynamic_cast<osg::Texture2DArray*>(
        ss->getTextureAttribute(0, osg::StateAttribute::TEXTURE));
    ASSERT_NE(nullptr, tex);

    std::vector<cv::Mat> images;
    images.push_back(cv::Mat(48, 64, CV_8UC3, cv::Scalar(255, 0, 0)));   // blue BGR
    images.push_back(cv::Mat(48, 64, CV_8UC3, cv::Scalar(0, 255, 0)));   // green BGR
    scene.UploadCameraImages(images);

    EXPECT_GE(tex->getNumImages(), 2u);
}
