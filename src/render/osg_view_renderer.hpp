#pragma once

//

#include "render/stitch_scene.hpp"

#include <osg/Camera>
#include <osg/Group>
#include <osg/Image>
#include <osg/ref_ptr>
#include <osgViewer/Viewer>

#include <opencv2/core.hpp>

#include <algorithm>
#include <map>
#include <string>
#include <vector>

namespace avm_stitch {

enum class RenderMode { Offscreen, Windowed };

struct ViewPose {
    std::string name;
    cv::Vec3d position;
    cv::Vec3d target;
    cv::Vec3d up;
    double fovy_deg = 60.0;
};

inline std::vector<ViewPose> DefaultViewPoses(const cv::Vec2d& car_center = cv::Vec2d(0.0, 0.0),
                                              double scale = 1.0) {
    const double cx = car_center[0];
    const double cy = car_center[1];
    const double z_ground = 0.2 * scale;       return {
        {"3d_front", {cx + 5.8 * scale, cy, 2.0 * scale},  {cx, cy, z_ground}, {0, 0, 1}, 60.0},
        {"3d_rear",  {cx - 5.8 * scale, cy, 2.0 * scale},  {cx, cy, z_ground}, {0, 0, 1}, 60.0},
        {"3d_left",  {cx, cy + 5.8 * scale, 2.0 * scale},  {cx, cy, z_ground}, {0, 0, 1}, 60.0},
        {"3d_right", {cx, cy - 5.8 * scale, 2.0 * scale},  {cx, cy, z_ground}, {0, 0, 1}, 60.0},
    };
}

struct ClipPlanes {
    double near;
    double far;
};

inline ClipPlanes ComputeClipPlanes(double scene_radius, double max_eye_to_target) {
    constexpr double kFarMargin = 5.3553572528096565;
    constexpr double kNearRel = 0.01;
    const double far = kFarMargin * (max_eye_to_target + scene_radius);
    const double near = std::clamp(scene_radius * kNearRel, 1.0e-3, far / 1000.0);
    return {near, far};
}

class OSGViewRenderer {
public:
    OSGViewRenderer(const StitchScene& scene,
                    double scene_radius,
                    const std::vector<ViewPose>& view_poses,
                    int output_width, int output_height,
                    RenderMode mode = RenderMode::Offscreen,
                    const cv::Vec2d& car_center = cv::Vec2d(0.0, 0.0));
    ~OSGViewRenderer() = default;
    OSGViewRenderer(const OSGViewRenderer&) = delete;
    OSGViewRenderer& operator=(const OSGViewRenderer&) = delete;

            std::map<std::string, cv::Mat> RenderAllViews();

            void WaitForWindowClose();

private:
    void SetupWindowed(osg::ref_ptr<osg::Group> root);
    void SetupOffscreenCameras(const StitchScene& scene,
                               const std::vector<ViewPose>& view_poses);
    void EnsureRealized();

    osg::ref_ptr<osgViewer::Viewer> viewer_;
    std::vector<osg::ref_ptr<osg::Camera>> view_cameras_;
    std::vector<osg::ref_ptr<osg::Image>> view_images_;
    std::vector<std::string> view_names_;
    int output_width_ = 960;
    int output_height_ = 640;
    bool realized_ = false;
    RenderMode mode_ = RenderMode::Offscreen;
    double scene_radius_ = 0.0;
    cv::Vec2d car_center_{0.0, 0.0};
};

}
