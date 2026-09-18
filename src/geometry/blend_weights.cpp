#include "geometry/blend_weights.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace avm_stitch {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

double NormalizeAngleDeg(double deg) {
    deg = std::fmod(deg, 360.0);
    if (deg < 0.0) deg += 360.0;
    return deg;
}

double AzimuthDeg(const CameraExtrinsics& e) {
    // Optical-axis ground projection azimuth: camera Z-axis (column 2 of R_wc)
    // projected onto the XY plane. Replaces old atan2(t[1], t[0]) which used
    // camera position azimuth (wrong for non-centric camera layouts).
    // R(0,2)=axis.x, R(1,2)=axis.y. atan2(0,0) returns 0.0 (optical axis up).
    return NormalizeAngleDeg(std::atan2(e.R(1,2), e.R(0,2)) * kRadToDeg);
}

// Shortest angular distance on the circle between two angles (degrees).
// dist = |Normalize(a - b)|; if > 180, wrap to 360 - dist.
double ShortestArcDeg(double a, double b) {
    double dist = std::abs(NormalizeAngleDeg(a - b));
    if (dist > 180.0) dist = 360.0 - dist;
    return dist;
}

// Midpoint boundary angle between two sorted azimuths (handles wrap-around).
double SectorBoundaryDeg(double az1, double az2) {
    if (az2 < az1) az2 += 360.0;
    return NormalizeAngleDeg((az1 + az2) / 2.0);
}

// (azimuth, original_index) pairs sorted by azimuth.
std::vector<std::pair<double, int>> SortCamerasByAzimuth(
    const std::vector<CameraExtrinsics>& extrinsics) {
    std::vector<std::pair<double, int>> sorted;
    const int N = static_cast<int>(extrinsics.size());
    sorted.reserve(N);
    for (int i = 0; i < N; ++i) {
        sorted.emplace_back(AzimuthDeg(extrinsics[i]), i);
    }
    std::sort(sorted.begin(), sorted.end());
    return sorted;
}

} // namespace

double ComputeBlendHalfAngle(int num_cameras) {
    if (num_cameras <= 0) return 0.0;
    return 52.0 / num_cameras;   // 13 deg * (360/N) / 90
}

std::vector<double> ComputeBlendWeights(
    const cv::Vec3d& P_world,
    const std::vector<CameraExtrinsics>& extrinsics,
    const cv::Vec2d& pole) {
    const int N = static_cast<int>(extrinsics.size());
    std::vector<double> weights(N, 0.0);
    if (N == 0) return weights;

    const double theta = NormalizeAngleDeg(
        std::atan2(P_world[1] - pole[1], P_world[0] - pole[0]) * kRadToDeg);
    auto sorted = SortCamerasByAzimuth(extrinsics);
    if (N == 1) {
        weights[sorted[0].second] = 1.0;
        return weights;
    }

    const double blend_half = ComputeBlendHalfAngle(N);

    // Nearest camera by azimuth.
    int best = 0;
    double best_dist = 360.0;
    for (int j = 0; j < N; ++j) {
        double d = ShortestArcDeg(theta, sorted[j].first);
        if (d < best_dist) {
            best_dist = d;
            best = j;
        }
    }

    // If theta is within the blend band of one of the two boundaries adjacent
    // to the nearest camera, blend the two cameras sharing that boundary.
    // The nearest camera owns the near side, so its weight grows 0.5 -> 1 as
    // theta moves from the boundary into its sector (cosine smoothstep).
    auto try_blend = [&](int a) -> bool {
        const int b = (a + 1) % N;
        const double boundary = SectorBoundaryDeg(sorted[a].first, sorted[b].first);
        const double dist = ShortestArcDeg(theta, boundary);
        if (dist > blend_half) return false;
        const double t = 0.5 + dist / (2.0 * blend_half);
        const double w_near = 0.5 * (1.0 - std::cos(t * kPi));
        const double w_far = 1.0 - w_near;
        if (best == a) {
            weights[sorted[a].second] = w_near;
            weights[sorted[b].second] = w_far;
        } else {
            weights[sorted[b].second] = w_near;
            weights[sorted[a].second] = w_far;
        }
        return true;
    };

    if (try_blend((best - 1 + N) % N)) return weights;
    if (try_blend(best)) return weights;

    // Outside any blend band: nearest camera owns the point outright.
    weights[sorted[best].second] = 1.0;
    return weights;
}

} // namespace avm_stitch
