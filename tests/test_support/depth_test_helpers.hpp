#pragma once

#include "camera_model/camera_extrinsics.hpp"

#include <opencv2/core.hpp>

#include <cmath>
#include <utility>

namespace avm_stitch::test {

constexpr double kDepthPi = 3.14159265358979323846;

inline double DegToRad(double deg) { return deg * kDepthPi / 180.0; }
inline double RadToDeg(double rad) { return rad * 180.0 / kDepthPi; }

inline cv::Matx33d MakeLookAtRotation(const cv::Vec3d& pos,
                                      const cv::Vec3d& target) {
    cv::Vec3d f = target - pos;
    f /= cv::norm(f);
    cv::Vec3d world_up(0.0, 0.0, 1.0);
    cv::Vec3d r = world_up.cross(f);
    if (cv::norm(r) < 1e-9)
        r = cv::Vec3d(1.0, 0.0, 0.0);
    r /= cv::norm(r);
    cv::Vec3d u = f.cross(r);
    return cv::Matx33d(r[0], u[0], f[0],
                       r[1], u[1], f[1],
                       r[2], u[2], f[2]);
}

inline std::pair<CameraExtrinsics, CameraExtrinsics> MakeStereoPair(double elevation = 0.0) {
    const double cPitch = std::cos(DegToRad(35.0));
    const double sPitch = std::sin(DegToRad(35.0));
    const double cYaw = std::cos(DegToRad(15.0));
    const double sYaw = std::sin(DegToRad(15.0));

    cv::Vec3d ta(0.0, 0.0, elevation);
    cv::Vec3d fa(cPitch, 0.0, -sPitch);
    CameraExtrinsics a{MakeLookAtRotation(ta, ta + fa), ta};

    cv::Vec3d tb(1.1, 0.0, elevation);
    cv::Vec3d fb(cPitch * cYaw, cPitch * sYaw, -sPitch);
    CameraExtrinsics b{MakeLookAtRotation(tb, tb + fb), tb};

    return {a, b};
}

inline std::pair<CameraExtrinsics, CameraExtrinsics> MakeLookDownPair() {
    auto look_down = [](const cv::Vec3d& pos) {
        cv::Vec3d f(0.0, 0.0, -1.0);
        cv::Vec3d r(1.0, 0.0, 0.0);
        cv::Vec3d u = f.cross(r);
        cv::Matx33d R(r[0], u[0], f[0],
                      r[1], u[1], f[1],
                      r[2], u[2], f[2]);
        return CameraExtrinsics{R, pos};
    };
    return {look_down(cv::Vec3d(0.0, 0.0, 1.0)),
            look_down(cv::Vec3d(1.0, 0.0, 1.0))};
}

}
