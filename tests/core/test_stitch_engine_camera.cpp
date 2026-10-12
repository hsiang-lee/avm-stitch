// Tests for StitchEngine::Configure(const std::vector<Camera>&, ...) — the
// multi-model configuration path (spec D1, plan batch A / Task A3).
// Validation tests are GL-free (Configure builds the scene but the OSG viewer
// context is created lazily on the first RenderFrame); the legacy-wrapper
// render check reuses the shared OSGGLTest gating.

#include <gtest/gtest.h>

#include "core/stitch_engine.hpp"
#include "geometry/camera_model.hpp"
#include "geometry/kb_camera_model.hpp"

#include "test_support/gl_fixture.hpp"
#include "test_support/stitch_test_helpers.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace avm_stitch;
using avm_stitch::test::OSGGLTest;

namespace {

Camera MakeKbCamera(double az_deg) {
    Camera cam;
    cam.extrinsics = avm_stitch::test::MakeLookAtCamera(az_deg, 4.0, 2.0, 0.0);
    cam.model = std::make_unique<KbCameraModel>(avm_stitch::test::MakeIntrinsics());
    return cam;
}

std::vector<Camera> MakeKbCameras(size_t n, int width = 640, int height = 480) {
    std::vector<Camera> cams;
    for (size_t i = 0; i < n; ++i) {
        Camera cam;
        cam.extrinsics = avm_stitch::test::MakeLookAtCameras(
            static_cast<int>(n))[i % n];
        CameraIntrinsics intr = avm_stitch::test::MakeIntrinsics();
        intr.width = width;
        intr.height = height;
        cam.model = std::make_unique<KbCameraModel>(intr);
        cams.push_back(std::move(cam));
    }
    return cams;
}

} // namespace

TEST(ConfigureCameras, N0_Throws) {
    StitchEngine engine;
    try {
        engine.Configure(std::vector<Camera>{}, avm_stitch::test::MakeCarBowl());
        FAIL() << "expected std::invalid_argument for N=0";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find("N must be in [1, 8]"),
                  std::string::npos)
            << "got: " << e.what();
    }
}

TEST(ConfigureCameras, N9_Throws) {
    StitchEngine engine;
    try {
        engine.Configure(MakeKbCameras(9), avm_stitch::test::MakeCarBowl());
        FAIL() << "expected std::invalid_argument for N=9";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find("N must be in [1, 8]"),
                  std::string::npos)
            << "got: " << e.what();
    }
}

TEST(ConfigureCameras, NullModel_Throws) {
    StitchEngine engine;
    std::vector<Camera> cams;
    cams.push_back(MakeKbCamera(0.0));
    Camera null_cam;
    null_cam.extrinsics = cams[0].extrinsics;
    null_cam.model = nullptr;
    cams.push_back(std::move(null_cam));

    try {
        engine.Configure(std::move(cams), avm_stitch::test::MakeCarBowl());
        FAIL() << "expected std::invalid_argument for null model";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find("model"), std::string::npos)
            << "got: " << e.what();
    }
}

TEST(ConfigureCameras, ImageSizeMismatch_Throws) {
    StitchEngine engine;
    std::vector<Camera> cams;
    cams.push_back(MakeKbCamera(0.0));   // 640x480
    {
        Camera cam;
        cam.extrinsics = avm_stitch::test::MakeLookAtCamera(180.0, 4.0, 2.0, 0.0);
        CameraIntrinsics intr = avm_stitch::test::MakeIntrinsics();
        intr.width = 1280;
        intr.height = 960;
        cam.model = std::make_unique<KbCameraModel>(intr);
        cams.push_back(std::move(cam));
    }

    try {
        engine.Configure(std::move(cams), avm_stitch::test::MakeCarBowl());
        FAIL() << "expected std::invalid_argument for mismatched image sizes";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find("same image size"),
                  std::string::npos)
            << "got: " << e.what();
    }
}

TEST(ConfigureCameras, KbConfigures) {
    StitchEngine engine;
    engine.Configure(MakeKbCameras(4), avm_stitch::test::MakeCarBowl());
    EXPECT_TRUE(engine.IsConfigured());
    EXPECT_EQ(4, engine.GetNumCameras());
}

TEST_F(OSGGLTest, LegacyWrapperStillConfigures) {
    StitchEngine engine;
    std::vector<CameraIntrinsics> intrs(4, avm_stitch::test::MakeIntrinsics());
    auto extrs = avm_stitch::test::MakeLookAtCameras(4);
    engine.Configure(intrs, extrs, avm_stitch::test::MakeCarBowl());
    ASSERT_TRUE(engine.IsConfigured());

    auto frames = avm_stitch::test::MakeColourFrames(4);
    auto views = engine.RenderFrame(frames);
    // 4 side views (topdown removed from default view poses).
    EXPECT_EQ(4, views.size());
}
