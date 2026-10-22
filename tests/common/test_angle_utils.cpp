#include <gtest/gtest.h>

#include "common/angle_utils.hpp"

using namespace avm_stitch;

TEST(AngleUtilsTest, NormalizeAngleDeg_KeepsInRangeUnchanged) {
    EXPECT_DOUBLE_EQ(0.0, NormalizeAngleDeg(0.0));
    EXPECT_DOUBLE_EQ(180.0, NormalizeAngleDeg(180.0));
    EXPECT_DOUBLE_EQ(357.0, NormalizeAngleDeg(357.0));
}

TEST(AngleUtilsTest, NormalizeAngleDeg_WrapsExactMultipleOf360ToZero) {
    EXPECT_DOUBLE_EQ(0.0, NormalizeAngleDeg(360.0));
    EXPECT_DOUBLE_EQ(0.0, NormalizeAngleDeg(720.0));
}

TEST(AngleUtilsTest, NormalizeAngleDeg_NegativeAdds360) {
    EXPECT_DOUBLE_EQ(315.0, NormalizeAngleDeg(-45.0));
    EXPECT_DOUBLE_EQ(180.0, NormalizeAngleDeg(-180.0));
}

TEST(AngleUtilsTest, NormalizeAngleDeg_NegativeExactMultipleOf360ToZero) {
    EXPECT_DOUBLE_EQ(0.0, NormalizeAngleDeg(-360.0));
    EXPECT_DOUBLE_EQ(0.0, NormalizeAngleDeg(-720.0));
}

TEST(AngleUtilsTest, NormalizeAngleDeg_MultiTurnWrapsIntoRange) {
    EXPECT_DOUBLE_EQ(30.0, NormalizeAngleDeg(750.0));
    EXPECT_DOUBLE_EQ(305.0, NormalizeAngleDeg(-415.0));
    EXPECT_DOUBLE_EQ(0.0, NormalizeAngleDeg(1080.0));
}

TEST(AngleUtilsTest, NormalizeAngleDeg_NonIntegerDegreesPreserved) {
    EXPECT_DOUBLE_EQ(123.456, NormalizeAngleDeg(123.456));
    EXPECT_DOUBLE_EQ(10.5, NormalizeAngleDeg(370.5));
}

TEST(AngleUtilsTest, Constants_HaveExpectedValues) {
    EXPECT_DOUBLE_EQ(3.14159265358979323846, kPi);
    EXPECT_DOUBLE_EQ(0.017453292519943295, kDegToRad);
    EXPECT_DOUBLE_EQ(57.29577951308232, kRadToDeg);
}

TEST(AngleUtilsTest, Constants_ConvertSymmetric) {
    EXPECT_NEAR(180.0, kRadToDeg * kPi, 1e-12);
    EXPECT_NEAR(kPi, kDegToRad * 180.0, 1e-12);
}
