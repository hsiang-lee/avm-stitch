#pragma once

// Lightweight, OSG-free test helpers shared by the pure-geometry suites
// (test_blend_weights, test_bowl_model) and the heavier renderer suites
// (via stitch_test_helpers.hpp).

#include "geometry/bowl_model.hpp"

#include <opencv2/core.hpp>

#include <cmath>
#include <vector>

namespace avm_stitch::test {

// N cameras at azimuths (deg), radius 3m, height 1m.
// Each camera's optical axis (forward) points in the azimuth direction
// (horizontal, NOT identity rotation): the bowl rim and flat centre
// become visible to the side cameras.
//   forward = (cos(az), sin(az), 0)       // column 2, horizontal
//   down    = (0, 0, -1)                   // column 1, camera Y axis
//   right   = down × forward               // column 0
inline std::vector<CameraExtrinsics> MakeSideCameras(
    const std::vector<double>& azimuths_deg) {
    std::vector<CameraExtrinsics> cams;
    for (double az : azimuths_deg) {
        double rad = az * M_PI / 180.0;
        cv::Vec3d pos(std::cos(rad) * 3.0, std::sin(rad) * 3.0, 1.0);
        cv::Vec3d forward(std::cos(rad), std::sin(rad), 0.0);
        cv::Vec3d down(0.0, 0.0, -1.0);
        cv::Vec3d right = down.cross(forward);
        cv::Matx33d R(right[0], down[0], forward[0],
                      right[1], down[1], forward[1],
                      right[2], down[2], forward[2]);
        cams.push_back({R, pos});
    }
    return cams;
}

} // namespace avm_stitch::test
