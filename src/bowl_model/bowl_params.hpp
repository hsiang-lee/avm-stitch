#pragma once

//

#include <opencv2/core.hpp>

#include <array>
#include <vector>

namespace avm_stitch {

enum class BowlType { BlockBoat };

class BowlParams {
public:
            BowlParams() = default;

            BowlParams(double car_length, double car_width, double round_l, double step,
               double r_outter, double k_slop, double z_exp, double max_z,
               cv::Vec2d car_center = cv::Vec2d(0.0, 0.0),
               BowlType type = BowlType::BlockBoat);

                    double step() const { return step_; }
    double r_outter() const { return r_outter_; }
        double round_l() const { return round_l_; }
            double k_slop() const { return k_slop_; }
    double z_exp() const { return z_exp_; }
        double max_z() const { return max_z_; }
        cv::Vec2d car_center() const { return car_center_; }

        double x_half() const { return car_length_ / 2.0 + round_l_; }
    double y_half() const { return car_width_ / 2.0 + round_l_; }
            double rd(double x, double y) const;
                double bowl_z(double rd) const;
    int grid_segments() const;
                        void GenerateBlockBoatMesh(std::vector<cv::Vec3d>& vertices,
                               std::vector<cv::Vec3d>& normals,
                               std::vector<std::array<int, 3>>& triangles) const;

                    BowlParams with_car_length(double v) const {
        BowlParams c = *this; c.car_length_ = v; return c;
    }
    BowlParams with_car_width(double v) const {
        BowlParams c = *this; c.car_width_ = v; return c;
    }
    BowlParams with_round_l(double v) const {
        BowlParams c = *this; c.round_l_ = v; return c;
    }
    BowlParams with_step(double v) const {
        BowlParams c = *this; c.step_ = v; return c;
    }
    BowlParams with_r_outter(double v) const {
        BowlParams c = *this; c.r_outter_ = v; return c;
    }
    BowlParams with_k_slop(double v) const {
        BowlParams c = *this; c.k_slop_ = v; return c;
    }
    BowlParams with_z_exp(double v) const {
        BowlParams c = *this; c.z_exp_ = v; return c;
    }
    BowlParams with_car_center(cv::Vec2d v) const {
        BowlParams c = *this; c.car_center_ = v; return c;
    }

private:
    double car_length_ = 0.0;
    double car_width_ = 0.0;
    double round_l_ = 0.0;
    double step_ = 0.0;
    double r_outter_ = 0.0;
    double k_slop_ = 0.0;
    double z_exp_ = 0.0;
    double max_z_ = 0.0;
    BowlType type_ = BowlType::BlockBoat;
    cv::Vec2d car_center_{0.0, 0.0};                                                                                                };

inline constexpr double kCalibratedAnchorRoutter = 12.6;

BowlParams DeriveBowlParams(double car_length, double car_width, double axis_x);

}
