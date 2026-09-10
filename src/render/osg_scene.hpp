#pragma once

// OSG scene graph construction for the AVM stitch engine:
//   - BuildOSGGeometry: bowl model -> osg::Geometry with per-vertex
//     UV (locations 2-5) and blend-weight (locations 6-7) attributes.
//   - Shader sources (GLSL 330, N-camera unrolled mapping shader).
//   - CreateCameraTextureArray / UploadCameraImages: N camera frames.
//   - BuildOSGScene: geode + stateset (program, uniforms, texture).
// Structural construction only — no GL context required.

#include "geometry/bowl_model.hpp"

#include <osg/Geometry>
#include <osg/Group>
#include <osg/Texture2DArray>

#include <opencv2/core.hpp>

#include <vector>

namespace avm_stitch {

/// Build the bowl model geometry with per-vertex attributes.
/// Vertex positions/normals are converted VCS -> OpenGL: (x,y,z) -> (x,z,-y).
/// @param bowl  generated + baked bowl model (precondition: bowl.IsBaked(),
///              num_cameras is taken from bowl.baked_num_cameras())
osg::ref_ptr<osg::Geometry> BuildOSGGeometry(const BowlModel& bowl);

/// GLSL 330 vertex shader source (N-camera mapping shader).
const char* GetVertexShaderSource();

/// GLSL 330 fragment shader source (N-camera unrolled mapping shader).
const char* GetFragmentShaderSource();

/// Create a layered texture array for N camera images.
osg::ref_ptr<osg::Texture2DArray> CreateCameraTextureArray(
    int num_layers, int w, int h);

/// Upload N BGR888 images (one per camera layer) into the texture array.
void UploadCameraImages(osg::Texture2DArray* tex,
                        const std::vector<cv::Mat>& images);

/// Build the complete scene graph: Group containing the bowl Geode with the
/// mapping program, uniforms and camera texture array stateset.
/// @param bowl  generated + baked bowl model (num_cameras from
///              bowl.baked_num_cameras(), image size from the caller)
osg::ref_ptr<osg::Group> BuildOSGScene(
    const BowlModel& bowl, int image_w, int image_h);

} // namespace avm_stitch
