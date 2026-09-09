// Regression smoke test: the avm_stitch public API is usable standalone
// (public headers self-contained, public API works as documented).

#include <gtest/gtest.h>

#include "geometry/bowl_model.hpp"
#include "geometry/projection.hpp"
#include "util/view_config.hpp"
#include "core/stitch_engine.hpp"
#include "core/stitch_backend.hpp"

#include <nlohmann/json.hpp>

#include <stdexcept>
#include <string>

using namespace avm_stitch;

TEST(RegressionTest, PublicApiHeadersAreSelfContained) {
    // Compiling this translation unit already proves the avm_stitch public
    // header tree is self-contained and the exported API links correctly.
    auto bowl = BowlParams::Default();
    EXPECT_EQ(5.6, bowl.car_length);

    auto poses = DefaultViewPoses();
    EXPECT_EQ(5u, poses.size());
}

TEST(RegressionTest, StitchEngineValidationStillGuardsInput) {
    // Cheap guard check without GL: Configure with zero cameras throws.
    StitchEngine engine;
    EXPECT_THROW(engine.Configure({}, {}, BowlParams::Default()),
                 std::invalid_argument);
    EXPECT_FALSE(engine.IsConfigured());
}

TEST(RegressionTest, ProjectionChainStillWorks) {
    // Projection is reused by the bake + engine; keep it healthy.
    CameraIntrinsics intr{640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
    CameraExtrinsics extr{cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)};
    double u, v;
    ProjectWorldToFishEye(cv::Vec3d(0, 0, 1), intr, extr, u, v);
    EXPECT_NEAR(320.0, u, 1e-6);
    EXPECT_NEAR(240.0, v, 1e-6);
}

TEST(StitchBackendFactory, RendererNameReturnsNullptrAfterMigration) {
    // The renderer proxy moved to avm-worker (its bridge library); this
    // library's factory no longer knows that name (D5).
    EXPECT_EQ(CreateStitchBackend("renderer"), nullptr);
}

TEST(StitchBackendFactory, OsgNameReturnsBackend) {
    // Factory positive path: the only name this library implements.
    auto backend = CreateStitchBackend("osg");
    ASSERT_NE(backend, nullptr);
}

TEST(StitchBackendFactory, RunWithoutImagesReturnsErrorEnvelope) {
    // Error-envelope contract, no GL needed: a task without 'images' fails
    // in OSGBackend::Run before any rendering happens.
    auto backend = CreateStitchBackend("osg");
    ASSERT_NE(backend, nullptr);
    const nlohmann::json result = backend->Run(nlohmann::json::object());
    EXPECT_EQ("stitch_error", result.value("type", ""));
    EXPECT_NE(std::string::npos,
              result.value("error", "").find("Missing 'images'"));
}
