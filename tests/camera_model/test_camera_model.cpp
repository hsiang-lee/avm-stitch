#include <gtest/gtest.h>

#include "camera_model/camera.hpp"
#include "camera_model/camera_extrinsics.hpp"
#include "camera_model/kb_camera_model.hpp"

#include <bit>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

using namespace avm_stitch;

namespace {

KbParams MakeKbParams(int width = 640, int height = 480) {
    KbParams p;
    p.width = width;
    p.height = height;
    p.fx = 300.0;
    p.fy = 300.0;
    p.cx = 320.0;
    p.cy = 240.0;
    return p;
}

}
TEST(MakeCameraModel, Kb_ReturnsModel) {
    auto model = MakeCameraModel("kb", MakeKbParams());
    ASSERT_NE(model, nullptr);
    EXPECT_EQ(640, model->width());
    EXPECT_EQ(480, model->height());
}

TEST(MakeCameraModel, UnknownModel_Throws) {
    try {
        auto model = MakeCameraModel("fov", MakeKbParams());
        (void)model;
        FAIL() << "expected std::invalid_argument for unknown model";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find("fov"), std::string::npos)
            << "what() should name the unknown model, got: " << e.what();
    }
}

TEST(MakeCameraModel, VariantMismatch_Throws) {
    try {
        auto model = MakeCameraModel("kb", LutParams{});
        (void)model;
        FAIL() << "expected std::invalid_argument for variant mismatch";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find("kb"), std::string::npos)
            << "what() should name the requested model, got: " << e.what();
    }
}

TEST(MakeCameraModel, NonPositiveSize_Throws) {
    try {
        auto model = MakeCameraModel("kb", MakeKbParams(/*width=*/0, /*height=*/480));
        (void)model;
        FAIL() << "expected std::invalid_argument for non-positive size";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find("width"), std::string::npos)
            << "what() should mention width, got: " << e.what();
    }
}
