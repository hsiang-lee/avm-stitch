#pragma once

//

#include "render/stitch_scene.hpp"

#include "bowl_model/bowl_params.hpp"

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

enum class RenderMode { Offscreen,
                        Windowed };

class OSGViewRenderer {
  public:
    OSGViewRenderer(const StitchScene& scene,
                    const BowlParams& bowl,
                    int output_width, int output_height,
                    RenderMode mode = RenderMode::Offscreen);
    ~OSGViewRenderer() = default;
    OSGViewRenderer(const OSGViewRenderer&) = delete;
    OSGViewRenderer& operator=(const OSGViewRenderer&) = delete;

    std::map<std::string, cv::Mat> RenderAllViews();

    void WaitForWindowClose();

  private:
    void SetupWindowed(osg::ref_ptr<osg::Group> root);
    void SetupOffscreenCameras(const StitchScene& scene);
    void EnsureRealized();

    osg::ref_ptr<osgViewer::Viewer> viewer_;
    std::vector<osg::ref_ptr<osg::Camera>> view_cameras_;
    std::vector<osg::ref_ptr<osg::Image>> view_images_;
    std::vector<std::string> view_names_;
    int output_width_ = 960;
    int output_height_ = 640;
    bool realized_ = false;
    RenderMode mode_ = RenderMode::Offscreen;
    BowlParams bowl_;
};

}
