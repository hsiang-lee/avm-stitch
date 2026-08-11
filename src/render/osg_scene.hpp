#pragma once

// OSG scene graph construction for the AVM stitch engine:
//   - BuildOSGGeometry: bowl mesh -> osg::Geometry with per-vertex
//     UV (locations 2-5) and blend-weight (locations 6-7) attributes.
//   - Shader sources (GLSL 330, N-camera unrolled mapping shader).
//   - CreateCameraTextureArray / UploadCameraImages: N camera frames.
//   - BuildOSGScene: geode + stateset (program, uniforms, texture).
// Structural construction only — no GL context required.

#include "geometry/bowl_model.hpp"
#include "geometry/mesh_bake.hpp"

#include <osg/Geometry>
#include <osg/Group>
#include <osg/Texture2DArray>

#include <opencv2/core.hpp>

#include <vector>

namespace avm_worker { namespace avm_stitch {

/// Build the bowl mesh geometry with per-vertex attributes.
/// Vertex positions/normals are converted VCS -> OpenGL: (x,y,z) -> (x,z,-y).
/// @param bowl         generated bowl mesh (VCS coordinates)
/// @param baked        per-vertex baked UV + weights (same vertex count)
/// @param num_cameras  number of cameras N (1..8)
osg::ref_ptr<osg::Geometry> BuildOSGGeometry(
    const BowlMesh& bowl, const BakedMesh& baked, int num_cameras);

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
osg::ref_ptr<osg::Group> BuildOSGScene(
    const BowlMesh& bowl, const BakedMesh& baked,
    int num_cameras, int image_w, int image_h);

}} // namespace avm_worker::avm_stitch
