// Regression smoke test: the avm_stitch public API is usable standalone
// (public headers self-contained, public API works as documented).

#include <gtest/gtest.h>

#include "avm_stitch/geometry/bowl_model.hpp"
#include "avm_stitch/geometry/projection.hpp"
#include "avm_stitch/util/view_config.hpp"
#include "avm_stitch/core/stitch_engine.hpp"

#include <stdexcept>

using namespace avm_worker;
using namespace avm_worker::avm_stitch;

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
