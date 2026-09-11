#pragma once

// Scene graph owner for the AVM stitch engine:
//   - bowl Geode (geometry + per-vertex UV/weight attributes, mapping shader,
//     uniforms) with the camera texture array bound in its stateset.
//   - shader sources (GLSL 330, N-camera unrolled mapping shader).
//   - UploadCameraImages: N camera frames into the texture array layers.
// Construction invariant: scene built and texture bound — ready to render.
// Structural construction only — no GL context required.

#include "geometry/bowl_model.hpp"

#include <osg/Geode>
#include <osg/Group>
#include <osg/Texture2DArray>

#include <opencv2/core.hpp>

#include <vector>

namespace avm_stitch {

/// Owns and manages the OSG scene graph for the stitch engine.
/// Construction invariant: scene built and texture bound — ready to render.
/// Structural construction only — no GL context required.
class StitchScene {
public:
    /// Build the complete scene: Group containing the bowl Geode (geometry +
    /// per-vertex UV/weight attributes, mapping shader, uniforms) with the
    /// camera texture array bound in its stateset.
    /// @param bowl  baked bowl model (num_cameras from bowl.baked_num_cameras())
    StitchScene(const BowlModel& bowl, int image_w, int image_h);

    /// Root node — the renderer attaches per-view cameras under it.
    osg::ref_ptr<osg::Group> root() const;

    /// The bowl geometry drawable shared by all per-view cameras
    /// (each view camera wraps it in its own Geode — single-parent rule).
    osg::ref_ptr<osg::Drawable> bowl_drawable() const;

    /// The bowl stateset (mapping program + uniforms + texture) shared
    /// alongside the drawable.
    osg::ref_ptr<osg::StateSet> bowl_stateset() const;

    /// Upload N BGR888 camera frames (one per texture layer).
    void UploadCameraImages(const std::vector<cv::Mat>& images);

private:
    osg::ref_ptr<osg::Group> root_;
    osg::ref_ptr<osg::Texture2DArray> texture_;
    osg::ref_ptr<osg::Geode> bowl_geode_;   // held to expose drawable/stateset
};

} // namespace avm_stitch
