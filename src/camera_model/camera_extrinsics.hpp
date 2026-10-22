#pragma once

// (OpenCV convention, camera y-axis points down). Behaviour-rich value type
// for the 3D world (VCS) -> fisheye pixel projection chain (header-only).
//
// intrinsic model (projection polynomial + effective-circle gates). That
// model was deleted in the camera refactor — the kb projection polynomial
// moved verbatim into KbCameraModel::ProjectCameraPoint (kb_camera_model.cpp)
// and the gating methods (FisheyeCircleRadius / InsideFisheyeCircle /
// EdgeFeather) into the CameraModel base (camera_model.cpp); kEps moved to
// camera_model.hpp. This file now holds extrinsics only.

#include "common/angle_utils.hpp"

#include <cmath>
#include <opencv2/core.hpp>

namespace avm_stitch {

class CameraExtrinsics {
  public:
    CameraExtrinsics() : R_(cv::Matx33d::eye()), t_(0.0, 0.0, 0.0) {}

    CameraExtrinsics(const cv::Matx33d& R_wc, const cv::Vec3d& t_wc)
        : R_(R_wc), t_(t_wc) {}

    cv::Vec3d ToCamera(const cv::Vec3d& P_world) const {
        return R_.t() * (P_world - t_);
    }

    cv::Vec3d ToWorld(const cv::Vec3d& P_cam) const {
        return R_ * P_cam + t_;
    }

    cv::Vec3d RotateWorldToCamera(const cv::Vec3d& v) const {
        return R_.t() * v;
    }
    cv::Vec3d RotateCameraToWorld(const cv::Vec3d& v) const {
        return R_ * v;
    }

    const cv::Vec3d& position() const { return t_; }

    double AzimuthDeg() const {
        const cv::Vec3d axis = RotateCameraToWorld(cv::Vec3d(0, 0, 1));
        return NormalizeAngleDeg(std::atan2(axis[1], axis[0]) * kRadToDeg);
    }

    cv::Matx33d RelativeRotationFrom(const CameraExtrinsics& other) const {
        return R_.t() * other.R_;
    }

  private:
    cv::Matx33d R_;
    cv::Vec3d t_;
};

}
