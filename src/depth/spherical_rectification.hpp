#pragma once
#include "camera_model/camera_extrinsics.hpp"
#include <opencv2/core.hpp>

//
//

namespace avm_stitch::depth {

struct RectificationBasis {
    cv::Vec3d e1, e2, e3;
    cv::Matx33d m1;       cv::Matx33d m2;       double baseline = 0.0;
};

RectificationBasis MakeRectificationBasis(const CameraExtrinsics& a,
                                          const CameraExtrinsics& b);

struct SphericalGrid {
    int width = 1440;                int height = 720;                double psi_min_deg = 0.0;
    double psi_max_deg = 180.0;
    bool mirror_rows = false;        double PsiDeg(int col) const;
    double PhiDeg(int row) const;
    cv::Vec3d DirectionC(int row, int col) const;   };

bool SkyUpNeedsMirror(const RectificationBasis& basis, const CameraExtrinsics& a, int height = 720);

cv::Vec3d CameraDirection(const RectificationBasis& basis, const cv::Vec3d& dir_c, bool second);

cv::Vec3d RectifiedDirection(const RectificationBasis& basis, const cv::Vec3d& dir_world,
                             const CameraExtrinsics& extr, bool second);

double PhiDeg(const cv::Vec3d& dir_c);

}
