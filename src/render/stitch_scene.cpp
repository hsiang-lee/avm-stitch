#include "render/stitch_scene.hpp"

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Program>
#include <osg/Shader>
#include <osg/StateSet>
#include <osg/Uniform>

#include <opencv2/imgproc.hpp>

#include <cstring>
#include <string>

namespace avm_stitch {

namespace {

constexpr int kUvSlotBase = 2;
constexpr int kNumUvSlots = 4;
constexpr int kWeightSlotBase = 6;
constexpr int kNumWeightSlots = 2;

constexpr std::array<float, 2> kZeroUv{0.0f, 0.0f};

template <typename Accessor>
osg::ref_ptr<osg::Vec4Array> BuildPackedVec4(const BowlModel& bowl,
                                             Accessor accessor) {
    auto arr = new osg::Vec4Array;
    arr->reserve(bowl.vertex_count());
    for (int i = 0; i < bowl.vertex_count(); ++i) {
        arr->push_back(accessor(bowl.baked_vertex(i)));
    }
    return arr;
}

osg::ref_ptr<osg::Geometry> BuildBowlGeometry(const BowlModel& bowl) {
    auto geom = new osg::Geometry;

    auto pos_array = new osg::Vec3Array;
    pos_array->reserve(bowl.vertex_count());
    for (int i = 0; i < bowl.vertex_count(); ++i) {
        const cv::Vec3d& v = bowl.vertex(i);
        pos_array->push_back(osg::Vec3(v[0], v[2], -v[1]));
    }
    geom->setVertexArray(pos_array);

    auto normal_array = new osg::Vec3Array;
    normal_array->reserve(bowl.normal_count());
    for (int i = 0; i < bowl.normal_count(); ++i) {
        const cv::Vec3d& n = bowl.normal(i);
        normal_array->push_back(osg::Vec3(n[0], n[2], -n[1]));
    }
    geom->setNormalArray(normal_array);
    geom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

    const int n = bowl.baked_num_cameras();
    for (int slot = 0; slot < kNumUvSlots; ++slot) {
        int c0 = slot * 2;
        int c1 = c0 + 1;
        auto uv_array = BuildPackedVec4(bowl, [n, c0, c1](const BowlModel::BakedVertex& bv) {
            const std::array<float, 2>& u0 =
                (c0 < n) ? bv.uv[static_cast<size_t>(c0)] : kZeroUv;
            const std::array<float, 2>& u1 =
                (c1 < n) ? bv.uv[static_cast<size_t>(c1)] : kZeroUv;
            return osg::Vec4(u0[0], u0[1], u1[0], u1[1]);
        });
        geom->setVertexAttribArray(kUvSlotBase + slot, uv_array);
        geom->setVertexAttribBinding(kUvSlotBase + slot,
                                     osg::Geometry::BIND_PER_VERTEX);
    }

    for (int slot = 0; slot < kNumWeightSlots; ++slot) {
        int c0 = slot * 4;
        auto w_array = BuildPackedVec4(bowl, [n, c0](const BowlModel::BakedVertex& bv) {
            auto w = [n, &bv](int c) -> float {
                return (c < n) ? bv.weight[static_cast<size_t>(c)] : 0.0f;
            };
            return osg::Vec4(w(c0), w(c0 + 1), w(c0 + 2), w(c0 + 3));
        });
        geom->setVertexAttribArray(kWeightSlotBase + slot, w_array);
        geom->setVertexAttribBinding(kWeightSlotBase + slot,
                                     osg::Geometry::BIND_PER_VERTEX);
    }

    auto indices = new osg::DrawElementsUInt(GL_TRIANGLES);
    indices->reserve(bowl.triangle_count() * 3);
    for (int t = 0; t < bowl.triangle_count(); ++t) {
        const std::array<int, 3> tri = bowl.triangle(t);
        indices->push_back(static_cast<unsigned int>(tri[0]));
        indices->push_back(static_cast<unsigned int>(tri[1]));
        indices->push_back(static_cast<unsigned int>(tri[2]));
    }
    geom->addPrimitiveSet(indices);

    return geom;
}

const char* VertexShaderSource() {
    return R"(#version 330 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexNormal;
layout(location = 2) in vec4 uvCoord0;   layout(location = 3) in vec4 uvCoord1;   layout(location = 4) in vec4 uvCoord2;   layout(location = 5) in vec4 uvCoord3;   layout(location = 6) in vec4 weights0;   layout(location = 7) in vec4 weights1;
uniform mat4 osg_ModelViewProjectionMatrix;
uniform int uNumCameras;

out vec3 fragNormal;
out vec4 fragUV[4];      out vec4 fragWeights0;   out vec4 fragWeights1;   flat out int fragNumCameras;

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

const char* FragmentShaderSource() {
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

osg::ref_ptr<osg::Texture2DArray> CreateCameraTextures(
    int num_layers, int w, int h) {
    auto tex = new osg::Texture2DArray;
    tex->setTextureSize(w, h, num_layers);
    tex->setInternalFormat(GL_RGBA8);
    tex->setSourceFormat(GL_BGRA);
    tex->setSourceType(GL_UNSIGNED_BYTE);
    tex->setResizeNonPowerOfTwoHint(false);
    tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
    tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    return tex;
}

}
StitchScene::StitchScene(const BowlModel& bowl, int image_w, int image_h) {
    root_ = new osg::Group;

    num_cameras_ = bowl.baked_num_cameras();
    auto geom = BuildBowlGeometry(bowl);

    bowl_geode_ = new osg::Geode;
    bowl_geode_->addDrawable(geom);

    texture_ = CreateCameraTextures(num_cameras_, image_w, image_h);

    auto program = new osg::Program;
    program->addShader(new osg::Shader(osg::Shader::VERTEX, VertexShaderSource()));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT, FragmentShaderSource()));
    program->addBindAttribLocation("uvCoord0", 2);
    program->addBindAttribLocation("uvCoord1", 3);
    program->addBindAttribLocation("uvCoord2", 4);
    program->addBindAttribLocation("uvCoord3", 5);
    program->addBindAttribLocation("weights0", 6);
    program->addBindAttribLocation("weights1", 7);

    auto state_set = bowl_geode_->getOrCreateStateSet();
    state_set->setAttributeAndModes(program, osg::StateAttribute::ON);
    state_set->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    state_set->addUniform(new osg::Uniform("uNumCameras", num_cameras_));
    state_set->addUniform(new osg::Uniform("uCameraTextures", 0));
    state_set->setTextureAttributeAndModes(0, texture_, osg::StateAttribute::ON);

    root_->addChild(bowl_geode_);
}

osg::ref_ptr<osg::Group> StitchScene::root() const {
    return root_;
}

osg::ref_ptr<osg::Drawable> StitchScene::bowl_drawable() const {
    return bowl_geode_->getNumDrawables() > 0 ? bowl_geode_->getDrawable(0)
                                              : nullptr;
}

osg::ref_ptr<osg::StateSet> StitchScene::bowl_stateset() const {
    return bowl_geode_->getStateSet();
}

void StitchScene::UploadCameraImages(const std::vector<cv::Mat>& images) {
    for (int i = 0; i < static_cast<int>(images.size()); ++i) {
        cv::Mat bgra;
        cv::cvtColor(images[i], bgra, cv::COLOR_BGR2BGRA);

        auto img = new osg::Image;
        img->allocateImage(bgra.cols, bgra.rows, 1, GL_BGRA, GL_UNSIGNED_BYTE);
        img->setInternalTextureFormat(GL_RGBA8);
        std::memcpy(img->data(), bgra.data, bgra.total() * bgra.elemSize());
        texture_->setImage(i, img);
    }
    texture_->dirtyTextureObject();
}

}
