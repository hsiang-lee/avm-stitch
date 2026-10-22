#pragma once

#include "bowl_model/bowl_model.hpp"
#include "camera_model/camera.hpp"
#include "camera_model/kb_camera_model.hpp"

#include <opencv2/core.hpp>

#include <cmath>
#include <memory>
#include <vector>

namespace avm_stitch::test {

// construction — the retired hand-written intrinsic model never existed here.
inline KbParams MakeKbParams() {
    return {640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
}

inline constexpr int kRealImageWidth = 1920;
inline constexpr int kRealImageHeight = 1536;

inline KbParams MakeRealIntrinsics() {
    return {kRealImageWidth, kRealImageHeight, 441.45, 441.45, 952.14, 758.27,
            0, 0, 0, 0};
}

inline CameraExtrinsics MakeIdentityExtrinsics() {
    return {cv::Matx33d::eye(), cv::Vec3d(0, 0, 0)};
}

inline KbParams MakeKbIntrinsics(double k1 = 0, double k2 = 0,
                                 double k3 = 0, double k4 = 0,
                                 double fx = 300.0, double fy = 300.0) {
    KbParams p;
    p.width = 640;
    p.height = 480;
    p.fx = fx;
    p.fy = fy;
    p.cx = 320.0;
    p.cy = 240.0;
    p.k1 = k1;
    p.k2 = k2;
    p.k3 = k3;
    p.k4 = k4;
    return p;
}

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

inline std::vector<Camera> WrapKbSideCameras(
    const std::vector<double>& azimuths_deg) {
    const KbParams params = MakeKbParams();
    std::vector<Camera> cams;
    for (const CameraExtrinsics& ext : MakeSideCameras(azimuths_deg)) {
        cams.emplace_back(ext, MakeCameraModel("kb", params));
    }
    return cams;
}

}
