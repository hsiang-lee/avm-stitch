// Tests for view name → AVMDisplayMode mapping.
// Header-only dependency (view_mapping.hpp), pure string mapping, no GL.

#include <gtest/gtest.h>

#include "core/view_mapping.hpp"

#include <string>

using namespace avm_worker::avm_stitch;

TEST(ViewMapping, MapsTopdownToTopDownAndAerial) {
    EXPECT_EQ(MapViewNameToDisplayMode("topdown"), kTopDownAndAerial);
}

TEST(ViewMapping, Maps3dFrontToD3FrontAndAerial) {
    EXPECT_EQ(MapViewNameToDisplayMode("3d_front"), kD3FrontAndAerial);
}

TEST(ViewMapping, Maps3dRearToD3RearAndAerial) {
    EXPECT_EQ(MapViewNameToDisplayMode("3d_rear"), kD3RearAndAerial);
}

TEST(ViewMapping, Maps3dLeftToD3LeftAndAerial) {
    EXPECT_EQ(MapViewNameToDisplayMode("3d_left"), kD3LeftAndAerial);
}

TEST(ViewMapping, Maps3dRightToD3RightAndAerial) {
    EXPECT_EQ(MapViewNameToDisplayMode("3d_right"), kD3RightAndAerial);
}

TEST(ViewMapping, ReturnsInvalidForUnknownViewName) {
    EXPECT_EQ(MapViewNameToDisplayMode("bogus"), kInvalidDisplayMode);
}

TEST(ViewMapping, ReturnsInvalidForEmptyViewName) {
    EXPECT_EQ(MapViewNameToDisplayMode(""), kInvalidDisplayMode);
}
