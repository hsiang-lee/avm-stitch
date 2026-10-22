#pragma once

#include "bowl_model/bowl_model.hpp"

#include <osg/Geode>
#include <osg/Group>
#include <osg/Texture2DArray>

#include <opencv2/core.hpp>

#include <vector>

namespace avm_stitch {

class StitchScene {
  public:
    StitchScene(const BowlModel& bowl, int image_w, int image_h);

    osg::ref_ptr<osg::Group> root() const;

    osg::ref_ptr<osg::Drawable> bowl_drawable() const;

    osg::ref_ptr<osg::StateSet> bowl_stateset() const;

    int num_cameras() const { return num_cameras_; }

    void UploadCameraImages(const std::vector<cv::Mat>& images);

  private:
    osg::ref_ptr<osg::Group> root_;
    osg::ref_ptr<osg::Texture2DArray> texture_;
    osg::ref_ptr<osg::Geode> bowl_geode_;
    int num_cameras_ = 0;
};

}
