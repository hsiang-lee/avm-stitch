#pragma once

#include <opencv2/core.hpp>

#include <array>
#include <string>
#include <vector>

namespace avm_stitch {

struct ViewPose {
    std::string name;
    cv::Vec3d position;
    cv::Vec3d target;
    cv::Vec3d up;
    double fovy_deg = 60.0;
};

struct ClipPlanes {
    double near;
    double far;
};

class BowlParams {
  public:
    BowlParams() = default;

    BowlParams(double car_length, double car_width, double flat_platform_radius,
               double grid_step, double outer_radius, double slope_coefficient,
               double height_exponent, double max_height,
               cv::Vec2d car_center = cv::Vec2d(0.0, 0.0));

    static BowlParams ForCarSize(double car_length, double car_width,
                                 double axis_x);

    double GridStep() const { return grid_step_; }
    double OuterRadius() const { return outer_radius_; }
    double FlatPlatformRadius() const { return flat_platform_radius_; }
    double SlopeCoefficient() const { return slope_coefficient_; }
    double HeightExponent() const { return height_exponent_; }
    double MaxHeight() const { return max_height_; }
    cv::Vec2d CarCenter() const { return car_center_; }

    double DistanceFromFlatPlatform(double x, double y) const;
    double BowlHeight(double distance_from_flat_platform) const;
    int GridSegmentsPerAxis() const;
    std::vector<ViewPose> DefaultViewPoses() const;
    ClipPlanes ClipPlanesFor(double max_eye_to_target) const;
    ViewPose HomeViewPose() const;
    double HomeViewDistance() const;
    void GenerateBlockBoatMesh(std::vector<cv::Vec3d>& vertices,
                               std::vector<cv::Vec3d>& normals,
                               std::vector<std::array<int, 3>>& triangles) const;

  private:
    double car_length_ = 0.0;
    double car_width_ = 0.0;
    double flat_platform_radius_ = 0.0;
    double grid_step_ = 0.0;
    double outer_radius_ = 0.0;
    double slope_coefficient_ = 0.0;
    double height_exponent_ = 0.0;
    double max_height_ = 0.0;
    cv::Vec2d car_center_{0.0, 0.0};
};

inline constexpr double kCalibratedAnchorOuterRadius = 12.6;

}
