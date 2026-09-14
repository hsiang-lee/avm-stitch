#pragma once

// Offscreen / windowed multi-view rendering with OpenSceneGraph:
//   - Construction: creates a viewer with either a headless EGL context
//     (Offscreen) or a real X11 window (Windowed), plus one FBO camera per
//     view pose (children of root).
//   - RenderAllViews: renders every view into its FBO and reads back RGBA
//     pixels as BGR cv::Mat (960x640 default).
//
// Default AVM view poses (pure math, header-only).

#include "render/stitch_scene.hpp"

#include <osg/Camera>
#include <osg/Group>
#include <osg/Image>
#include <osg/ref_ptr>
#include <osgViewer/Viewer>

#include <opencv2/core.hpp>

#include <map>
#include <string>
#include <vector>

namespace avm_stitch {

/// Rendering mode for OSGViewRenderer.
/// - Offscreen (default): headless EGL surfaceless GL context, no window.
/// - Windowed: real X11 window with trackball controls (requires DISPLAY).
enum class RenderMode { Offscreen, Windowed };

struct ViewPose {
    std::string name;
    cv::Vec3d position;   // VCS, metres
    cv::Vec3d target;
    cv::Vec3d up;
    double fovy_deg = 60.0;
};

// Default AVM view poses (pure math, header-only): the 5-view frame
// (topdown + 4 side views). Dimensions are the historically calibrated
// car-scale values; the scale factor multiplies every car-relative offset
// AND the absolute camera/target height so the frame covers vehicles of
// any size (callers pass r_outter/12.6 == the DeriveBowlParams scale s).
// scale=1.0 reproduces the exact hardcoded car frame.
inline std::vector<ViewPose> DefaultViewPoses(const cv::Vec2d& car_center = cv::Vec2d(0.0, 0.0),
                                              double scale = 1.0) {
    const double cx = car_center[0];
    const double cy = car_center[1];
    const double z_ground = 0.2 * scale;   // look-at target height over the flat bowl centre
    return {
        {"topdown",  {cx - 1.82 * scale, cy, 6.0 * scale}, {cx, cy, z_ground}, {0, 0, 1}, 60.0},
        {"3d_front", {cx + 5.8 * scale, cy, 2.0 * scale},  {cx, cy, z_ground}, {0, 0, 1}, 60.0},
        {"3d_rear",  {cx - 5.8 * scale, cy, 2.0 * scale},  {cx, cy, z_ground}, {0, 0, 1}, 60.0},
        {"3d_left",  {cx, cy + 5.8 * scale, 2.0 * scale},  {cx, cy, z_ground}, {0, 0, 1}, 60.0},
        {"3d_right", {cx, cy - 5.8 * scale, 2.0 * scale},  {cx, cy, z_ground}, {0, 0, 1}, 60.0},
    };
}

class OSGViewRenderer {
public:
    /// Build the renderer: creates an osgViewer::Viewer with either a headless
    /// EGL context (Offscreen) or a real X11 window (Windowed), plus one FBO
    /// camera per view pose (children of the scene root). Construction performs
    /// no GL rendering — viewer realize() stays lazy until first render.
    /// @param scene         scene graph owner (root + bowl drawable/stateset)
    /// @param view_poses    view poses to render (one FBO camera each)
    /// @param output_width  output image width
    /// @param output_height output image height
    /// @param mode          Offscreen (default) or Windowed
    OSGViewRenderer(const StitchScene& scene,
                    const std::vector<ViewPose>& view_poses,
                    int output_width, int output_height,
                    RenderMode mode = RenderMode::Offscreen,
                    const cv::Vec2d& car_center = cv::Vec2d(0.0, 0.0));
    ~OSGViewRenderer() = default;
    OSGViewRenderer(const OSGViewRenderer&) = delete;
    OSGViewRenderer& operator=(const OSGViewRenderer&) = delete;

    /// Render all views, one cv::Mat (BGR888) per view name.
    /// Realizes the GL context on first call.
    std::map<std::string, cv::Mat> RenderAllViews();

    /// Block until the viewer's window is closed (Windowed mode only).
    /// Returns immediately in Offscreen mode.
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
    cv::Vec2d car_center_{0.0, 0.0};
};

} // namespace avm_stitch
