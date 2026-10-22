
#include <gtest/gtest.h>

#include "camera_model/camera.hpp"
#include "camera_model/camera_extrinsics.hpp"
#include "camera_model/kb_camera_model.hpp"

#include "test_support/geometry_test_helpers.hpp"

#include <opencv2/core.hpp>

#include <memory>
#include <utility>

using namespace avm_stitch;

using avm_stitch::test::MakeIdentityExtrinsics;
using avm_stitch::test::MakeKbIntrinsics;

TEST(Camera, DefaultIsInvalid) {
    Camera cam;
    EXPECT_FALSE(cam.Valid());
}

TEST(Camera, ConstructedIsValidAndDelegates) {
    Camera cam(MakeIdentityExtrinsics(),
               std::make_unique<KbCameraModel>(MakeKbIntrinsics()));
    EXPECT_TRUE(cam.Valid());
    EXPECT_EQ(640, cam.Width());
    EXPECT_EQ(480, cam.Height());
    EXPECT_EQ(320.0, cam.Cx());
    EXPECT_EQ(240.0, cam.Cy());
}

TEST(Camera, MovePreservesValidity) {
    Camera src(MakeIdentityExtrinsics(),
               std::make_unique<KbCameraModel>(MakeKbIntrinsics()));
    ASSERT_TRUE(src.Valid());
    Camera dst = std::move(src);
    EXPECT_TRUE(dst.Valid());
    EXPECT_FALSE(src.Valid());
}
