#include "bowl_model/blend_weights.hpp"

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
                            const cv::Vec3d axis = e.RotateCameraToWorld(cv::Vec3d(0, 0, 1));
    return NormalizeAngleDeg(std::atan2(axis[1], axis[0]) * kRadToDeg);
}

double ShortestArcDeg(double a, double b) {
    double dist = std::abs(NormalizeAngleDeg(a - b));
    if (dist > 180.0) dist = 360.0 - dist;
    return dist;
}

double SectorBoundaryDeg(double az1, double az2) {
    if (az2 < az1) az2 += 360.0;
    return NormalizeAngleDeg((az1 + az2) / 2.0);
}

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

}
double ComputeBlendHalfAngle(int num_cameras) {
    if (num_cameras <= 0) return 0.0;
    return 52.0 / num_cameras;   }

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

        int best = 0;
    double best_dist = 360.0;
    for (int j = 0; j < N; ++j) {
        double d = ShortestArcDeg(theta, sorted[j].first);
        if (d < best_dist) {
            best_dist = d;
            best = j;
        }
    }

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

        weights[sorted[best].second] = 1.0;
    return weights;
}

}
