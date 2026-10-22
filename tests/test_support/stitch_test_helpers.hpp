#pragma once

//

#include "bowl_model/bowl_model.hpp"
#include "camera_model/camera.hpp"
#include "camera_model/kb_camera_model.hpp"
#include "geometry_test_helpers.hpp"
#include "render/stitch_scene.hpp"

#include <osg/Texture2DArray>

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace avm_stitch::test {

inline KbParams MakeIntrinsics() {
                        return MakeKbParams();
}

// N kb CameraModels (distinct heap objects — Configure transfers ownership of
// each model, so N distinct models are required even when geometry matches).
inline std::vector<std::unique_ptr<CameraModel>> MakeKbModels(
    int num_cameras, const KbParams& params = MakeKbParams()) {
    std::vector<std::unique_ptr<CameraModel>> models;
    models.reserve(static_cast<size_t>(num_cameras));
    for (int i = 0; i < num_cameras; ++i) {
        models.push_back(MakeCameraModel("kb", params));
    }
    return models;
}

inline BowlParams MakeCarBowl() {
    return DeriveBowlParams(5.6, 2.2, 2.9);
}

inline BowlParams MakeCoarseBowlParams(double r_outter) {
    return MakeCarBowl()
        .with_car_center(cv::Vec2d(0.0, 0.0))
        .with_step(1.0)                  .with_r_outter(r_outter);
}

inline std::vector<Camera> WrapKbCameras(const std::vector<KbParams>& intrs,
                                         const std::vector<CameraExtrinsics>& extrs) {
    std::vector<Camera> cams;
    cams.reserve(intrs.size());
    for (size_t i = 0; i < intrs.size(); ++i) {
        cams.emplace_back(extrs[i], MakeCameraModel("kb", intrs[i]));
    }
    return cams;
}

inline BowlModel MakeCoarseBowl(double r_outter,
                                const std::vector<KbParams>& intrs,
                                const std::vector<CameraExtrinsics>& extrs) {
    return BowlModel(MakeCoarseBowlParams(r_outter), WrapKbCameras(intrs, extrs));
}

inline std::vector<KbParams> DefaultIntrinsics() {
    return std::vector<KbParams>(4, MakeRealIntrinsics());
}

inline std::vector<CameraExtrinsics> DefaultBakeCameras() {
    return MakeSideCameras({0, 180, 90, -90});
}

inline std::vector<Camera> MakeDefaultKbCameras() {
    return WrapKbCameras(DefaultIntrinsics(), DefaultBakeCameras());
}

inline CameraExtrinsics MakeLookAtCamera(double az_deg, double r, double h,
                                         double target_z) {
    double rad = az_deg * M_PI / 180.0;
    cv::Vec3d pos(r * std::cos(rad), r * std::sin(rad), h);
    cv::Vec3d target(0.0, 0.0, target_z);
    cv::Vec3d f = target - pos;
    f = f / cv::norm(f);
    cv::Vec3d up(0.0, 0.0, 1.0);
    cv::Vec3d right = f.cross(up);
    right = right / cv::norm(right);
    cv::Vec3d u = right.cross(f);
    cv::Vec3d down = -u;           cv::Matx33d R(right[0], down[0], f[0],
                  right[1], down[1], f[1],
                  right[2], down[2], f[2]);
    return {R, pos};
}

inline std::vector<CameraExtrinsics> MakeLookAtCameras(int num_cameras) {
    std::vector<CameraExtrinsics> cams;
    for (int i = 0; i < num_cameras; ++i) {
        cams.push_back(MakeLookAtCamera(i * 360.0 / num_cameras, 4.0, 2.0, 0.0));
    }
    return cams;
}

struct TestScene {
    std::unique_ptr<StitchScene> scene;
    bool has_visible_vertices = false;
};

inline osg::ref_ptr<osg::Texture2DArray> SceneTexture(const StitchScene& scene) {
    return dynamic_cast<osg::Texture2DArray*>(
        scene.bowl_stateset()->getTextureAttribute(0, osg::StateAttribute::TEXTURE));
}

inline TestScene BuildTestSceneWithExtrinsics(
    int num_cameras, const std::vector<CameraExtrinsics>& extrs) {
    TestScene ts;
    std::vector<KbParams> intrs(num_cameras, MakeIntrinsics());
    auto model = MakeCoarseBowl(12.6, intrs, extrs);

    ts.has_visible_vertices = model.HasVisibleVertices();

    ts.scene = std::make_unique<StitchScene>(model, intrs[0].width, intrs[0].height);
    return ts;
}

inline TestScene BuildTestScene(int num_cameras) {
    return BuildTestSceneWithExtrinsics(num_cameras, MakeLookAtCameras(num_cameras));
}

inline std::vector<cv::Mat> MakeColourFrames(int num_cameras) {
    const cv::Scalar colours[4] = {
        cv::Scalar(0, 0, 255),             cv::Scalar(0, 255, 0),             cv::Scalar(255, 0, 0),             cv::Scalar(0, 255, 255),       };
    KbParams intr = MakeIntrinsics();
    std::vector<cv::Mat> frames;
    for (int i = 0; i < num_cameras; ++i) {
        frames.push_back(cv::Mat(intr.height, intr.width, CV_8UC3, colours[i % 4]));
    }
    return frames;
}

inline int CountVividPixels(const cv::Mat& img) {
    int vivid = 0;
    for (int y = 0; y < img.rows; ++y) {
        const uchar* row = img.ptr<uchar>(y);
        for (int x = 0; x < img.cols; ++x) {
            const uchar* px = row + x * 3;
            int mn = std::min({px[0], px[1], px[2]});
            int mx = std::max({px[0], px[1], px[2]});
            if (mx - mn > 100) ++vivid;
        }
    }
    return vivid;
}

}
