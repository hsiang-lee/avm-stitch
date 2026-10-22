
#include <gtest/gtest.h>

#include "bowl_model/bowl_model.hpp"
#include "render/osg_view_renderer.hpp"
#include "core/stitch_engine.hpp"

using namespace avm_stitch;

TEST(RegressionTest, PublicApiHeadersAreSelfContained) {
            const BowlParams bowl = DeriveBowlParams(5.6, 2.2, 2.9);
    EXPECT_DOUBLE_EQ(12.6, bowl.r_outter());

    auto poses = DefaultViewPoses();
    EXPECT_EQ(4u, poses.size());
}
