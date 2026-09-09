// Tests for avm_stitch OSG scene construction: geometry, shaders, textures.
// Structural tests only — pure CPU data structures, no GL context required.

#include <gtest/gtest.h>

#include "geometry/bowl_model.hpp"
#include "geometry/mesh_bake.hpp"
#include "render/osg_scene.hpp"

#include <osg/Geometry>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Program>
#include <osg/Texture2DArray>

#include <cmath>
#include <vector>

using namespace avm_stitch;

namespace {

CameraIntrinsics MakeIntrinsics() {
    return {1920, 1536, 441.45, 441.45, 952.14, 758.27, 0, 0, 0, 0};
}

// N identity-rotation cameras at azimuths (deg), radius 3m, height 1m.
std::vector<CameraExtrinsics> MakeSideCameras(
    const std::vector<double>& azimuths_deg) {
    std::vector<CameraExtrinsics> cams;
    for (double az : azimuths_deg) {
        double rad = az * M_PI / 180.0;
        cv::Vec3d pos(std::cos(rad) * 3.0, std::sin(rad) * 3.0, 1.0);
        cams.push_back({cv::Matx33d::eye(), pos});
    }
    return cams;
}

BowlMesh MakeCoarseBowl(double r_outter) {
    BowlParams bowl = BowlParams::Default();
    bowl.step = 1.0;        // coarse grid for test speed
    bowl.r_outter = r_outter;
    return CreateBowlMesh(bowl);
}

// Bake a coarse bowl for N side cameras. Returns a valid baked mesh with the
// same vertex count as the bowl (unused camera slots stay zero).
BakedMesh MakeBaked(int num_cameras) {
    auto mesh = MakeCoarseBowl(12.6);
    std::vector<CameraIntrinsics> intrs(num_cameras, MakeIntrinsics());
    std::vector<double> azimuths;
    for (int i = 0; i < num_cameras; ++i) azimuths.push_back(i * 360.0 / num_cameras);
    auto extrs = MakeSideCameras(azimuths);
    return BakeMesh(mesh, intrs, extrs);
}

} // namespace

TEST(OSGSceneTest, BuildOSGGeometry_PositionAttributeCorrect) {
    auto bowl = MakeCoarseBowl(5.0);
    auto baked = MakeBaked(4);
    auto geom = BuildOSGGeometry(bowl, baked, 4);
    ASSERT_TRUE(geom.valid());

    auto* va = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
    ASSERT_NE(nullptr, va);
    ASSERT_EQ(bowl.vertices.size(), va->size());

    // VCS -> OpenGL: (x, y, z) -> (x, z, -y)
    for (size_t i = 0; i < bowl.vertices.size(); ++i) {
        EXPECT_NEAR(bowl.vertices[i][0], (*va)[i].x(), 1e-6);
        EXPECT_NEAR(bowl.vertices[i][2], (*va)[i].y(), 1e-6);
        EXPECT_NEAR(-bowl.vertices[i][1], (*va)[i].z(), 1e-6);
    }
}

TEST(OSGSceneTest, BuildOSGGeometry_UVArraysExist) {
    auto bowl = MakeCoarseBowl(5.0);
    auto baked = MakeBaked(4);
    auto geom = BuildOSGGeometry(bowl, baked, 4);
    ASSERT_TRUE(geom.valid());
    for (int loc = 2; loc <= 5; ++loc) {
        EXPECT_NE(nullptr, geom->getVertexAttribArray(loc)) << "UV location " << loc;
    }
}

TEST(OSGSceneTest, BuildOSGGeometry_WeightArraysExist) {
    auto bowl = MakeCoarseBowl(5.0);
    auto baked = MakeBaked(4);
    auto geom = BuildOSGGeometry(bowl, baked, 4);
    ASSERT_TRUE(geom.valid());
    for (int loc = 6; loc <= 7; ++loc) {
        EXPECT_NE(nullptr, geom->getVertexAttribArray(loc)) << "weight location " << loc;
    }
}

TEST(OSGSceneTest, BuildOSGGeometry_IndexCount) {
    auto bowl = MakeCoarseBowl(5.0);
    auto baked = MakeBaked(4);
    auto geom = BuildOSGGeometry(bowl, baked, 4);
    ASSERT_TRUE(geom.valid());

    int index_count = 0;
    for (unsigned int k = 0; k < geom->getNumPrimitiveSets(); ++k) {
        auto* de = dynamic_cast<osg::DrawElementsUInt*>(geom->getPrimitiveSet(k));
        if (de) index_count += de->getNumIndices();
    }
    EXPECT_EQ(bowl.triangles.size() * 3u, static_cast<size_t>(index_count));
    EXPECT_GT(index_count, 0);
}

TEST(OSGSceneTest, BuildOSGGeometry_NormalBinding) {
    auto bowl = MakeCoarseBowl(5.0);
    auto baked = MakeBaked(4);
    auto geom = BuildOSGGeometry(bowl, baked, 4);
    ASSERT_TRUE(geom.valid());
    EXPECT_EQ(osg::Geometry::BIND_PER_VERTEX, geom->getNormalBinding());
    EXPECT_NE(nullptr, geom->getNormalArray());
}

TEST(OSGSceneTest, ShaderSources_VertexCompilesSyntactically) {
    const char* vs = GetVertexShaderSource();
    ASSERT_NE(nullptr, vs);
    std::string s(vs);
    EXPECT_NE(std::string::npos, s.find("#version 330 core"));
    EXPECT_NE(std::string::npos, s.find("layout(location = 0) in vec3 vertexPosition"));
    EXPECT_NE(std::string::npos, s.find("osg_ModelViewProjectionMatrix"));
    EXPECT_NE(std::string::npos, s.find("main()"));
}

TEST(OSGSceneTest, ShaderSources_FragmentCompilesSyntactically) {
    const char* fs = GetFragmentShaderSource();
    ASSERT_NE(nullptr, fs);
    std::string s(fs);
    EXPECT_NE(std::string::npos, s.find("#version 330 core"));
    EXPECT_NE(std::string::npos, s.find("sampler2DArray"));
    EXPECT_NE(std::string::npos, s.find("fragWeights0"));
    EXPECT_NE(std::string::npos, s.find("WEIGHT_MIN"));
    EXPECT_NE(std::string::npos, s.find("finalColor"));
}

TEST(OSGSceneTest, ShaderSources_All8CamerasUnrolled) {
    const char* fs = GetFragmentShaderSource();
    ASSERT_NE(nullptr, fs);
    std::string s(fs);
    for (int i = 0; i < 8; ++i) {
        EXPECT_NE(std::string::npos, s.find("Camera " + std::to_string(i)))
            << "fragment shader must unroll camera " << i;
    }
}

TEST(OSGSceneTest, CreateCameraTextureArray_SetsCorrectSize) {
    auto tex = CreateCameraTextureArray(4, 1920, 1536);
    ASSERT_TRUE(tex.valid());
    EXPECT_EQ(1920, tex->getTextureWidth());
    EXPECT_EQ(1536, tex->getTextureHeight());
    EXPECT_EQ(4, tex->getTextureDepth());
}

TEST(OSGSceneTest, BuildOSGScene_ReturnsNonNullGroup) {
    auto bowl = MakeCoarseBowl(5.0);
    auto baked = MakeBaked(4);
    auto root = BuildOSGScene(bowl, baked, 4, 1920, 1536);
    ASSERT_TRUE(root.valid());
    EXPECT_GE(root->getNumChildren(), 1u);
}

TEST(OSGSceneTest, BuildOSGScene_ProgramAttached) {
    auto bowl = MakeCoarseBowl(5.0);
    auto baked = MakeBaked(4);
    auto root = BuildOSGScene(bowl, baked, 4, 1920, 1536);
    ASSERT_TRUE(root.valid());
    ASSERT_GE(root->getNumChildren(), 1u);

    auto* geode = dynamic_cast<osg::Geode*>(root->getChild(0));
    ASSERT_NE(nullptr, geode);
    auto* ss = geode->getStateSet();
    ASSERT_NE(nullptr, ss);
    EXPECT_NE(nullptr, ss->getAttribute(osg::StateAttribute::PROGRAM));
}

TEST(OSGSceneTest, UploadCameraImages_DoesNotCrash) {
    auto tex = CreateCameraTextureArray(2, 64, 48);
    ASSERT_TRUE(tex.valid());

    std::vector<cv::Mat> images;
    images.push_back(cv::Mat(48, 64, CV_8UC3, cv::Scalar(255, 0, 0)));   // blue BGR
    images.push_back(cv::Mat(48, 64, CV_8UC3, cv::Scalar(0, 255, 0)));   // green BGR
    UploadCameraImages(tex.get(), images);

    EXPECT_GE(tex->getNumImages(), 2u);
}
