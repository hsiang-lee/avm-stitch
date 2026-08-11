#pragma once

// 3D world (VCS) -> fisheye pixel projection (header-only).
// Delegates the pinhole + fisheye distortion to cv::fisheye::projectPoints
// (OpenCV convention: T_wc extrinsics, camera y-axis points down).

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>

namespace avm_worker { namespace avm_stitch {

struct CameraIntrinsics {
    int width = 0, height = 0;
    double fx = 0, fy = 0, cx = 0, cy = 0;
    double k1 = 0, k2 = 0, k3 = 0, k4 = 0;
    double fisheye_radius = 0.0;   // 0 = auto (inscribed circle of the image)
};

struct CameraExtrinsics {
    // T_wc: camera -> world. R is 3x3 (row-major), t is translation (m).
    cv::Matx33d R;
    cv::Vec3d t;
};

constexpr double kEps = 1e-9;

inline cv::Vec3d TransformWorldToCamera(const cv::Vec3d& P_world,
                                        const CameraExtrinsics& extr) {
    // P_cam = R^T * (P_world - t)
    return extr.R.t() * (P_world - extr.t);
}

/// Project a 3D world point (VCS) to fisheye pixel coordinates.
/// @param P_world  3D point in VCS (metres)
/// @param intr     camera intrinsics (fx,fy,cx,cy,k1..k4)
/// @param extr     camera extrinsics T_wc (camera->world)
/// @param u, v     output fisheye pixel coordinates; (-1,-1) if behind camera
inline void ProjectWorldToFishEye(const cv::Vec3d& P_world,
                                  const CameraIntrinsics& intr,
                                  const CameraExtrinsics& extr,
                                  double& u, double& v) {
    cv::Vec3d P_cam = TransformWorldToCamera(P_world, extr);
    if (P_cam[2] <= kEps) {
        u = -1.0;   // behind camera
        v = -1.0;
        return;
    }

    // world -> camera rvec/tvec (T_wc -> T_cw)
    cv::Matx33d Rcw = extr.R.t();
    cv::Vec3d tcw = -Rcw * extr.t;
    cv::Mat rvec;
    cv::Rodrigues(cv::Mat(Rcw), rvec);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << tcw[0], tcw[1], tcw[2]);
    cv::Mat K = (cv::Mat_<double>(3, 3) << intr.fx, 0, intr.cx,
                 0, intr.fy, intr.cy, 0, 0, 1);
    cv::Mat D = (cv::Mat_<double>(4, 1) << intr.k1, intr.k2, intr.k3, intr.k4);
    std::vector<cv::Point3d> obj(1, cv::Point3d(P_world[0], P_world[1], P_world[2]));
    std::vector<cv::Point2d> img;
    cv::fisheye::projectPoints(obj, img, rvec, tvec, K, D);
    u = img[0].x;
    v = img[0].y;
}

/// Check if a world point is in front of the camera (Z_cam > 0).
inline bool IsForwardFacing(const cv::Vec3d& P_world, const CameraExtrinsics& extr) {
    return TransformWorldToCamera(P_world, extr)[2] > 0.0;
}

// Fisheye effective-circle radius. 0 in intrinsics -> auto: the inscribed
// circle centred at (cx,cy) (distance to the nearest image edge).
inline double FisheyeCircleRadius(const CameraIntrinsics& intr) {
    if (intr.fisheye_radius > 0.0) return intr.fisheye_radius;
    double r = std::min({intr.cx, intr.cy,
                         static_cast<double>(intr.width) - intr.cx,
                         static_cast<double>(intr.height) - intr.cy});
    return std::max(0.0, r);
}

// True when the projected pixel lies inside the fisheye effective circle
// (valid image content; outside is the black dead zone).
inline bool IsInsideFisheyeCircle(const CameraIntrinsics& intr,
                                  double u_px, double v_px) {
    double r_px = std::hypot(u_px - intr.cx, v_px - intr.cy);
    return r_px <= FisheyeCircleRadius(intr);
}

// Feather factor for pixels near the fisheye effective-circle edge: 1.0 well
// inside, smoothly -> 0 as the projected pixel approaches the circle boundary.
// Prevents linear texture filtering from mixing the black dead-zone into the
// seam blend. feather_px is the fade width in pixels (<= 0 -> no fade).
inline double FisheyeEdgeFeather(const CameraIntrinsics& intr,
                                 double u_px, double v_px,
                                 double feather_px = 16.0) {
    if (feather_px <= 0.0) return 1.0;
    const double r_px = std::hypot(u_px - intr.cx, v_px - intr.cy);
    const double radius = FisheyeCircleRadius(intr);
    const double margin = radius - r_px;          // >0 guaranteed inside circle
    if (margin >= feather_px) return 1.0;
    if (margin <= 0.0) return 0.0;
    return margin / feather_px;
}

}} // namespace avm_worker::avm_stitch
