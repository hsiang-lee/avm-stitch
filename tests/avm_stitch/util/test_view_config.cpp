// Tests for avm_stitch view config (default view poses).
// Pure math, no GL / OSG dependency.

#include <gtest/gtest.h>

#include "avm_stitch/util/view_config.hpp"

using namespace avm_worker::avm_stitch;

TEST(ViewConfigTest, DefaultViewPoses_Returns5Views) {
    auto poses = DefaultViewPoses();
    ASSERT_EQ(5u, poses.size());
}

TEST(ViewConfigTest, DefaultViewPoses_HasCorrectNames) {
    auto poses = DefaultViewPoses();
    ASSERT_EQ(5u, poses.size());
    EXPECT_EQ("topdown", poses[0].name);
    EXPECT_EQ("3d_front", poses[1].name);
    EXPECT_EQ("3d_rear", poses[2].name);
    EXPECT_EQ("3d_left", poses[3].name);
    EXPECT_EQ("3d_right", poses[4].name);
}

TEST(ViewConfigTest, DefaultViewPoses_TopdownPosition) {
    auto poses = DefaultViewPoses();
    ASSERT_GE(poses.size(), 1u);
    EXPECT_NEAR(-1.82, poses[0].position[0], 1e-9);
    EXPECT_NEAR(0.0, poses[0].position[1], 1e-9);
    EXPECT_NEAR(6.0, poses[0].position[2], 1e-9);
}

TEST(ViewConfigTest, DefaultViewPoses_AllFovy60) {
    auto poses = DefaultViewPoses();
    ASSERT_EQ(5u, poses.size());
    for (const auto& pose : poses) {
        EXPECT_DOUBLE_EQ(60.0, pose.fovy_deg);
    }
}

TEST(ViewConfigTest, DefaultViewPoses_AllTargetSame) {
    auto poses = DefaultViewPoses();
    ASSERT_EQ(5u, poses.size());
    for (const auto& pose : poses) {
        EXPECT_NEAR(0.0, pose.target[0], 1e-9);
        EXPECT_NEAR(0.0, pose.target[1], 1e-9);
        EXPECT_NEAR(0.2, pose.target[2], 1e-9);
        EXPECT_NEAR(0.0, pose.up[0], 1e-9);
        EXPECT_NEAR(0.0, pose.up[1], 1e-9);
        EXPECT_NEAR(1.0, pose.up[2], 1e-9);
    }
}
