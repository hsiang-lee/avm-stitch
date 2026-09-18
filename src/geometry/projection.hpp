#pragma once

// 3D world (VCS) -> fisheye pixel projection (header-only).
// Hand-written equidistant fisheye model (θ = atan2(r_xy, z) ∈ [0, π])
// replaces the previous cv::fisheye::projectPoints call.
// (OpenCV convention: T_wc extrinsics, camera y-axis points down).

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>

namespace avm_stitch {

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
/// Uses hand-written equidistant fisheye model (θ = atan2(r_xy, z) ∈ [0, π]),
/// replacing the previous cv::fisheye::projectPoints call. Removes z≤0 early
/// return — the model handles all z values, including z<0 (no mirror pixel).
/// @param P_world  3D point in VCS (metres)
/// @param intr     camera intrinsics (fx,fy,cx,cy,k1..k4)
/// @param extr     camera extrinsics T_wc (camera->world)
/// @param u, v     output fisheye pixel coordinates
inline void ProjectWorldToFishEye(const cv::Vec3d& P_world,
                                  const CameraIntrinsics& intr,
                                  const CameraExtrinsics& extr,
                                  double& u, double& v) {
    cv::Vec3d P_cam = TransformWorldToCamera(P_world, extr);
    const double x = P_cam[0], y = P_cam[1], z = P_cam[2];
    const double r_xy = std::sqrt(x*x + y*y);

    if (r_xy < kEps) {
        u = intr.cx;
        v = intr.cy;
        return;
    }

    double theta = std::atan2(r_xy, z);        // ∈ [0, π]
    double theta2 = theta * theta;
    double theta_d = theta * (1.0 + intr.k1 * theta2
                                   + intr.k2 * theta2 * theta2
                                   + intr.k3 * theta2 * theta2 * theta2
                                   + intr.k4 * theta2 * theta2 * theta2 * theta2);
    double factor = theta_d / r_xy;
    u = intr.cx + intr.fx * factor * x;
    v = intr.cy + intr.fy * factor * y;
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

// Hard gate for pixels near the fisheye effective-circle edge: 1.0 well
// inside, 0.0 within the feather band. Prevents the dark vignette at the
// fisheye rim (measured luminance profile: r220=71, r230=19, r240=0.15)
// from being sampled — the "feather" acts as a hard visibility gate, not a
// soft fade (since the dark rim is pure black, linear interpolation cannot
// salvage it). feather_px is the exclusion width in pixels (<= 0 -> no
// exclusion). Default 40 px covers the full vignette ramp.
inline double FisheyeEdgeFeather(const CameraIntrinsics& intr,
                                 double u_px, double v_px,
                                 double feather_px = 40.0) {
    if (feather_px <= 0.0) return 1.0;
    const double r_px = std::hypot(u_px - intr.cx, v_px - intr.cy);
    const double radius = FisheyeCircleRadius(intr);
    const double margin = radius - r_px;          // >0 guaranteed inside circle
    // Hard gate: exclude everything within feather_px of the circle edge
    // (was a linear fade before the 2026-09-11 fix — changed to a step
    // because the D3 fallback normalizes weight and amplifies even a
    // fractionally-safe sample to full weight, rendering the dark rim).
    if (margin >= feather_px) return 1.0;
    return 0.0;
}

} // namespace avm_stitch
