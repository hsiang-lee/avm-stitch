#pragma once

// Offscreen 5-view rendering with OpenSceneGraph:
//   - Initialize: creates a headless osgViewer::Viewer (EGL surfaceless GL
//     context, no window, no X server) and one FBO camera per view pose
//     (children of root).
//   - RenderAllViews: renders every view into its FBO and reads back RGBA
//     pixels as BGR cv::Mat (960x640 default).
// The EGL headless GL context is created eagerly in Initialize() (surfaceless
// EGL does not touch X), while the osgViewer viewer realize() still happens
// lazily on the first render, so Configure() performs no GL rendering.

#include "avm_stitch/util/view_config.hpp"

#include <osg/Camera>
#include <osg/Group>
#include <osg/Image>
#include <osg/ref_ptr>
#include <osgViewer/Viewer>

#include <opencv2/core.hpp>

#include <map>
#include <string>
#include <vector>

namespace avm_worker { namespace avm_stitch {

class OSGViewRenderer {
public:
    OSGViewRenderer() = default;
    ~OSGViewRenderer() = default;
    OSGViewRenderer(const OSGViewRenderer&) = delete;
    OSGViewRenderer& operator=(const OSGViewRenderer&) = delete;

    /// Prepare the offscreen renderer. Adds one FBO camera per view pose to
    /// `root` (the scene graph). Does not touch GL until the first render.
    /// @param root          scene root containing the bowl Geode
    /// @param view_poses    view poses to render (one FBO camera each)
    /// @param output_width  output image width (default 960)
    /// @param output_height output image height (default 640)
    void Initialize(osg::ref_ptr<osg::Group> root,
                    const std::vector<ViewPose>& view_poses,
                    int output_width, int output_height);

    /// Render all views, one cv::Mat (BGR888) per view name.
    /// Creates/realizes the GL context on first call.
    std::map<std::string, cv::Mat> RenderAllViews();

private:
    void EnsureRealized();

    osg::ref_ptr<osgViewer::Viewer> viewer_;
    std::vector<osg::ref_ptr<osg::Camera>> view_cameras_;
    std::vector<osg::ref_ptr<osg::Image>> view_images_;
    std::vector<std::string> view_names_;
    int output_width_ = 960;
    int output_height_ = 640;
    bool realized_ = false;
};

}} // namespace avm_worker::avm_stitch
