// Regression smoke test: the avm_stitch public API is usable standalone
// (public headers self-contained, public API works as documented).
// The former StitchEngineValidationStillGuardsInput and
// ProjectionChainStillWorks cases duplicated StitchEngineTest.ConfigureN0Throws
// (+ IsConfiguredFalseBefore) and ProjectionTest.IdentityCamera_Center and
// were removed.

#include <gtest/gtest.h>

#include "geometry/bowl_model.hpp"
#include "render/osg_view_renderer.hpp"
#include "core/stitch_engine.hpp"

using namespace avm_stitch;

TEST(RegressionTest, PublicApiHeadersAreSelfContained) {
    // Compiling this translation unit already proves the avm_stitch public
    // header tree is self-contained and the exported API links correctly.
    const BowlParams bowl = DeriveBowlParams(5.6, 2.2, 2.9);
    EXPECT_EQ(5.6, bowl.car_length);

    auto poses = DefaultViewPoses();
    EXPECT_EQ(5u, poses.size());
}
