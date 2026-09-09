#include "render/osg_scene.hpp"

#include <osg/Geode>
#include <osg/Program>
#include <osg/Shader>
#include <osg/StateSet>
#include <osg/Uniform>

#include <opencv2/imgproc.hpp>

#include <cstring>
#include <string>

namespace avm_stitch {

namespace {

// Fixed 8-camera slot layout (locations 2-5 = UV vec4, 6-7 = weights vec4).
constexpr int kUvSlotBase = 2;
constexpr int kNumUvSlots = 4;
constexpr int kWeightSlotBase = 6;
constexpr int kNumWeightSlots = 2;

// Build a per-vertex packed Vec4 attribute array (one vec4 per baked vertex).
template <typename Accessor>
osg::ref_ptr<osg::Vec4Array> BuildPackedVec4(const BakedMesh& baked,
                                             Accessor accessor) {
    auto arr = new osg::Vec4Array;
    arr->reserve(baked.vertices.size());
    for (const auto& bv : baked.vertices) {
        arr->push_back(accessor(bv));
    }
    return arr;
}

} // namespace

osg::ref_ptr<osg::Geometry> BuildOSGGeometry(
    const BowlMesh& bowl, const BakedMesh& baked, int num_cameras) {
    (void)num_cameras;
    auto geom = new osg::Geometry;

    // ---- vertex positions (location 0), VCS -> OpenGL: (x,y,z) -> (x,z,-y)
    auto pos_array = new osg::Vec3Array;
    pos_array->reserve(bowl.vertices.size());
    for (const auto& v : bowl.vertices) {
        pos_array->push_back(osg::Vec3(v[0], v[2], -v[1]));
    }
    geom->setVertexArray(pos_array);

    // ---- normals (location 1), same VCS -> OpenGL conversion
    auto normal_array = new osg::Vec3Array;
    normal_array->reserve(bowl.normals.size());
    for (const auto& n : bowl.normals) {
        normal_array->push_back(osg::Vec3(n[0], n[2], -n[1]));
    }
    geom->setNormalArray(normal_array);
    geom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

    // ---- UV attributes (locations 2-5): 4 vec4 slots, 2 cameras per slot
    for (int slot = 0; slot < kNumUvSlots; ++slot) {
        int c0 = slot * 2;
        int c1 = c0 + 1;
        auto uv_array = BuildPackedVec4(baked, [c0, c1](const BakedVertex& bv) {
            return osg::Vec4(bv.uv[c0][0], bv.uv[c0][1], bv.uv[c1][0], bv.uv[c1][1]);
        });
        geom->setVertexAttribArray(kUvSlotBase + slot, uv_array);
        geom->setVertexAttribBinding(kUvSlotBase + slot,
                                     osg::Geometry::BIND_PER_VERTEX);
    }

    // ---- weight attributes (locations 6-7): 2 vec4 slots, 4 cameras per slot
    for (int slot = 0; slot < kNumWeightSlots; ++slot) {
        int c0 = slot * 4;
        auto w_array = BuildPackedVec4(baked, [c0](const BakedVertex& bv) {
            return osg::Vec4(bv.weight[c0], bv.weight[c0 + 1],
                             bv.weight[c0 + 2], bv.weight[c0 + 3]);
        });
        geom->setVertexAttribArray(kWeightSlotBase + slot, w_array);
        geom->setVertexAttribBinding(kWeightSlotBase + slot,
                                     osg::Geometry::BIND_PER_VERTEX);
    }

    // ---- triangle indices
    auto indices = new osg::DrawElementsUInt(GL_TRIANGLES);
    indices->reserve(bowl.triangles.size() * 3);
    for (const auto& tri : bowl.triangles) {
        indices->push_back(static_cast<unsigned int>(tri[0]));
        indices->push_back(static_cast<unsigned int>(tri[1]));
        indices->push_back(static_cast<unsigned int>(tri[2]));
    }
    geom->addPrimitiveSet(indices);

    return geom;
}

const char* GetVertexShaderSource() {
    // N-camera mapping vertex shader (generalised mapping.vs).
    // NOTE: osg_ModelViewProjectionMatrix is not auto-updated by the vcpkg
    // osg GL2 build; OSGViewRenderer sets it per camera as a StateSet uniform.
    return R"(#version 330 core

// Vertex attributes (from osg::Geometry::setVertexAttribArray)
layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexNormal;
layout(location = 2) in vec4 uvCoord0;   // (u0, v0, u1, v1)
layout(location = 3) in vec4 uvCoord1;   // (u2, v2, u3, v3)
layout(location = 4) in vec4 uvCoord2;   // (u4, v4, u5, v5)
layout(location = 5) in vec4 uvCoord3;   // (u6, v6, u7, v7)
layout(location = 6) in vec4 weights0;   // (w0, w1, w2, w3)
layout(location = 7) in vec4 weights1;   // (w4, w5, w6, w7)

uniform mat4 osg_ModelViewProjectionMatrix;
uniform int uNumCameras;

out vec3 fragNormal;
out vec4 fragUV[4];      // uvCoord0..3 pass-through
out vec4 fragWeights0;   // weights0
out vec4 fragWeights1;   // weights1
flat out int fragNumCameras;

void main()
{
    fragNormal = vertexNormal;
    fragUV[0] = uvCoord0;
    fragUV[1] = uvCoord1;
    fragUV[2] = uvCoord2;
    fragUV[3] = uvCoord3;
    fragWeights0 = weights0;
    fragWeights1 = weights1;
    fragNumCameras = uNumCameras;

    gl_Position = osg_ModelViewProjectionMatrix * vec4(vertexPosition, 1.0);
}
)";
}

const char* GetFragmentShaderSource() {
    // N-camera mapping fragment shader: unrolled 8-camera blend (mapping.fs).
    return R"(#version 330 core

uniform sampler2DArray uCameraTextures;
uniform int uNumCameras;

in vec3 fragNormal;
in vec4 fragUV[4];
in vec4 fragWeights0;
in vec4 fragWeights1;
flat in int fragNumCameras;

out vec4 finalColor;

const float WEIGHT_MIN = 0.001;

void main()
{
    // Manually unrolled 8-camera blend (avoids dynamic sampler-array indexing)
    vec3 result = vec3(0.0, 0.0, 0.0);

    // Camera 0
    if (fragNumCameras > 0 && fragWeights0.x > WEIGHT_MIN) {
        result += texture(uCameraTextures, vec3(fragUV[0].xy, 0)).rgb * fragWeights0.x;
    }
    // Camera 1
    if (fragNumCameras > 1 && fragWeights0.y > WEIGHT_MIN) {
        result += texture(uCameraTextures, vec3(fragUV[0].zw, 1)).rgb * fragWeights0.y;
    }
    // Camera 2
    if (fragNumCameras > 2 && fragWeights0.z > WEIGHT_MIN) {
        result += texture(uCameraTextures, vec3(fragUV[1].xy, 2)).rgb * fragWeights0.z;
    }
    // Camera 3
    if (fragNumCameras > 3 && fragWeights0.w > WEIGHT_MIN) {
        result += texture(uCameraTextures, vec3(fragUV[1].zw, 3)).rgb * fragWeights0.w;
    }
    // Camera 4
    if (fragNumCameras > 4 && fragWeights1.x > WEIGHT_MIN) {
        result += texture(uCameraTextures, vec3(fragUV[2].xy, 4)).rgb * fragWeights1.x;
    }
    // Camera 5
    if (fragNumCameras > 5 && fragWeights1.y > WEIGHT_MIN) {
        result += texture(uCameraTextures, vec3(fragUV[2].zw, 5)).rgb * fragWeights1.y;
    }
    // Camera 6
    if (fragNumCameras > 6 && fragWeights1.z > WEIGHT_MIN) {
        result += texture(uCameraTextures, vec3(fragUV[3].xy, 6)).rgb * fragWeights1.z;
    }
    // Camera 7
    if (fragNumCameras > 7 && fragWeights1.w > WEIGHT_MIN) {
        result += texture(uCameraTextures, vec3(fragUV[3].zw, 7)).rgb * fragWeights1.w;
    }

    finalColor = vec4(result, 1.0);
}
)";
}

osg::ref_ptr<osg::Texture2DArray> CreateCameraTextureArray(
    int num_layers, int w, int h) {
    auto tex = new osg::Texture2DArray;
    tex->setTextureSize(w, h, num_layers);
    tex->setInternalFormat(GL_RGBA8);
    tex->setSourceFormat(GL_BGRA);     // OpenCV BGR -> BGRA (A=255)
    tex->setSourceType(GL_UNSIGNED_BYTE);
    // Do NOT rescale non-power-of-two camera frames: osg::Texture defaults
    // _resizeNonPowerOfTwoHint to true, which would scale e.g. 1920x1536 to
    // 2048x2048 and then refuse to upload (dimension mismatch -> black output).
    tex->setResizeNonPowerOfTwoHint(false);
    tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
    tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    return tex;
}

void UploadCameraImages(osg::Texture2DArray* tex,
                        const std::vector<cv::Mat>& images) {
    if (!tex) return;
    for (int i = 0; i < static_cast<int>(images.size()); ++i) {
        cv::Mat bgra;
        cv::cvtColor(images[i], bgra, cv::COLOR_BGR2BGRA);

        // Copy the pixels into an owned osg::Image. We deliberately do NOT use
        // NO_DELETE on the cv::Mat buffer: the cv::Mat is transient in the
        // RenderFrame call, so an owned copy keeps the texture layer valid.
        auto img = new osg::Image;
        img->allocateImage(bgra.cols, bgra.rows, 1, GL_BGRA, GL_UNSIGNED_BYTE);
        // Match the texture's internal format so the subload path accepts it.
        img->setInternalTextureFormat(GL_RGBA8);
        std::memcpy(img->data(), bgra.data, bgra.total() * bgra.elemSize());
        tex->setImage(i, img);
    }
    tex->dirtyTextureObject();
}

osg::ref_ptr<osg::Group> BuildOSGScene(
    const BowlMesh& bowl, const BakedMesh& baked,
    int num_cameras, int image_w, int image_h) {
    auto root = new osg::Group;

    auto geom = BuildOSGGeometry(bowl, baked, num_cameras);

    auto geode = new osg::Geode;
    geode->addDrawable(geom);

    auto texture = CreateCameraTextureArray(num_cameras, image_w, image_h);

    auto program = new osg::Program;
    program->addShader(new osg::Shader(osg::Shader::VERTEX, GetVertexShaderSource()));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT, GetFragmentShaderSource()));
    // Explicit attribute binding (shader layout qualifiers already match).
    program->addBindAttribLocation("uvCoord0", 2);
    program->addBindAttribLocation("uvCoord1", 3);
    program->addBindAttribLocation("uvCoord2", 4);
    program->addBindAttribLocation("uvCoord3", 5);
    program->addBindAttribLocation("weights0", 6);
    program->addBindAttribLocation("weights1", 7);

    auto state_set = geode->getOrCreateStateSet();
    state_set->setAttributeAndModes(program, osg::StateAttribute::ON);
    // Fixed-function lighting would shade the baked bowl; the mapping shader
    // outputs the blended texture colour directly.
    state_set->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    state_set->addUniform(new osg::Uniform("uNumCameras", num_cameras));
    state_set->addUniform(new osg::Uniform("uCameraTextures", 0));  // unit 0
    state_set->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);

    root->addChild(geode);
    return root;
}

} // namespace avm_stitch
