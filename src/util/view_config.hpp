#pragma once

// Default AVM view poses (pure math, header-only).

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace avm_stitch {

struct ViewPose {
    std::string name;
    cv::Vec3d position;   // VCS, metres
    cv::Vec3d target;
    cv::Vec3d up;
    double fovy_deg = 60.0;
};

inline std::vector<ViewPose> DefaultViewPoses() {
    return {
        {"topdown",  {-1.82, 0.0, 6.0}, {0, 0, 0.2}, {0, 0, 1}, 60.0},
        {"3d_front", {5.8, 0.0, 2.0},   {0, 0, 0.2}, {0, 0, 1}, 60.0},
        {"3d_rear",  {-5.8, 0.0, 2.0},  {0, 0, 0.2}, {0, 0, 1}, 60.0},
        {"3d_left",  {0, 5.8, 2.0},     {0, 0, 0.2}, {0, 0, 1}, 60.0},
        {"3d_right", {0, -5.8, 2.0},    {0, 0, 0.2}, {0, 0, 1}, 60.0},
    };
}

} // namespace avm_stitch
