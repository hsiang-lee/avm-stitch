// Tests for view name → AVMDisplayMode mapping.
// Header-only dependency (view_mapping.hpp), pure string mapping, no GL.

#include <gtest/gtest.h>

#include "core/view_mapping.hpp"

#include <string>

using namespace avm_worker::avm_stitch;

// Lock display-mode constants to avm-renderer 0.1.15's AVMDisplayMode enum
// values (AvmConstants.h): D3_REAR=14, D3_LEFT=16, D3_FRONT=18,
// D3_RIGHT=20, D3_TOP=22.
static_assert(kTopDownAndAerial == 22, "D3_TOP_AND_AERIAL must be 22");
static_assert(kD3FrontAndAerial == 18, "D3_FRONT_AND_AERIAL must be 18");
static_assert(kD3RearAndAerial  == 14, "D3_REAR_AND_AERIAL must be 14");
static_assert(kD3LeftAndAerial  == 16, "D3_LEFT_AND_AERIAL must be 16");
static_assert(kD3RightAndAerial == 20, "D3_RIGHT_AND_AERIAL must be 20");

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
