#include "depth/spherical_rectification.hpp"
#include "depth/depth_common.hpp"

#include <cmath>

namespace avm_stitch::depth {

namespace {

using detail::kDegToRad;
using detail::kRadToDeg;

cv::Vec3d NormalizeOr(const cv::Vec3d& v, const cv::Vec3d& fallback) {
    double n = cv::norm(v);
    return n > 0.0 ? v / n : fallback;
}

}
RectificationBasis MakeRectificationBasis(const CameraExtrinsics& a,
                                          const CameraExtrinsics& b) {
    RectificationBasis out;

        cv::Matx33d R = b.RelativeRotationFrom(a);

        cv::Vec3d t_ab = b.ToCamera(a.position());

            cv::Vec3d t_ba_a = -R.t() * t_ab;
    out.baseline = cv::norm(t_ba_a);
    out.e1 = NormalizeOr(t_ba_a, cv::Vec3d(1.0, 0.0, 0.0));

            cv::Vec3d zb = R.t() * cv::Vec3d(0.0, 0.0, 1.0);
    cv::Vec3d zm = NormalizeOr(cv::Vec3d(0.0, 0.0, 1.0) + zb, cv::Vec3d(0.0, 0.0, 1.0));
    cv::Vec3d e2_raw = zm - (zm.dot(out.e1)) * out.e1;
    if (cv::norm(e2_raw) < 1e-9) {
                        cv::Vec3d ref(0.0, 0.0, 1.0);
        if (std::abs(out.e1.dot(ref)) > 0.9) ref = cv::Vec3d(1.0, 0.0, 0.0);
        out.e2 = NormalizeOr(ref - (ref.dot(out.e1)) * out.e1, cv::Vec3d(0.0, 1.0, 0.0));
    } else {
        out.e2 = e2_raw / cv::norm(e2_raw);
    }

            out.e3 = out.e1.cross(out.e2);

        out.m1 = cv::Matx33d(out.e1[0], out.e1[1], out.e1[2],
                         out.e2[0], out.e2[1], out.e2[2],
                         out.e3[0], out.e3[1], out.e3[2]);
    out.m2 = out.m1 * R.t();

    return out;
}

double SphericalGrid::PsiDeg(int col) const {
            double t = width > 1 ? static_cast<double>(col) / (width - 1) : 0.0;
    return psi_min_deg + t * (psi_max_deg - psi_min_deg);
}

double SphericalGrid::PhiDeg(int row) const {
            double step = height > 1 ? 360.0 / (height - 1) : 0.0;
    double phi = -180.0 + row * step;
    if (mirror_rows) phi = -phi;
    return phi;
}

cv::Vec3d SphericalGrid::DirectionC(int row, int col) const {
        double psi = PsiDeg(col) * kDegToRad;
    double phi = PhiDeg(row) * kDegToRad;
    return cv::Vec3d(std::cos(psi),
                     std::sin(psi) * std::cos(phi),
                     std::sin(psi) * std::sin(phi));
}

bool SkyUpNeedsMirror(const RectificationBasis& basis, const CameraExtrinsics& a, int height) {
                cv::Vec3d sky(0.0, 0.0, 1.0);
    double phi = PhiDeg(RectifiedDirection(basis, sky, a, false));
    double half = height > 0 ? 0.5 * height : 360.0;
    double row = height > 1 ? (phi + 180.0) * (height - 1) / 360.0 : 0.0;
    return row >= half;
}

cv::Vec3d CameraDirection(const RectificationBasis& basis, const cv::Vec3d& dir_c, bool second) {
            return second ? basis.m2.t() * dir_c : basis.m1.t() * dir_c;
}

cv::Vec3d RectifiedDirection(const RectificationBasis& basis, const cv::Vec3d& dir_world,
                             const CameraExtrinsics& extr, bool second) {
        cv::Vec3d d_cam = extr.RotateWorldToCamera(dir_world);
    return second ? basis.m2 * d_cam : basis.m1 * d_cam;
}

double PhiDeg(const cv::Vec3d& dir_c) {
                double phi = std::atan2(dir_c[2], dir_c[1]) * kRadToDeg;
    if (phi <= -180.0) phi += 360.0;
    return phi;
}

}
