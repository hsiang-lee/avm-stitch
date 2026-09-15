#pragma once

// Shared test helpers for avm_stitch tests, moved verbatim from the
// tests/core/test_stitch_engine.cpp anonymous namespace (MakeBakedBowl was
// unified under its other copy's name MakeCoarseBowl). Header-only: include
// instead of keeping per-file private copies.
//
// NOTE: the band-frame flip-regression helpers (kBandColours / MakeBandFrame /
// MatchBand / DominantBand) and the view-dump helpers (MakeSkyGroundFrames /
// DumpViews / PrintColourRegionStats / PrintSkyGroundStats) are used only by
// test_stitch_engine.cpp and stay local to that file.

#include "geometry/bowl_model.hpp"
#include "render/stitch_scene.hpp"

#include <osg/Texture2DArray>

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace avm_stitch::test {

inline CameraIntrinsics MakeIntrinsics() {
    // 640x480 with a moderate focal length: a ring of bowl-rim vertices near
    // each side camera projects inside the image, so the bake has visible
    // cameras and the mapping shader has real texture work to do.
    return {640, 480, 300.0, 300.0, 320.0, 240.0, 0, 0, 0, 0};
}

// N cameras at azimuths (deg), radius 3m, height 1m.
// Each camera's optical axis (forward) points in the azimuth direction
// (horizontal, NOT identity rotation): the bowl rim and flat centre
// become visible to the side cameras.
// Rotation construction matches MakeNCameras in test_blend_weights.cpp:
//   forward = (cos(az), sin(az), 0)       // column 2, horizontal
//   down    = (0, 0, -1)                   // column 1, camera Y axis
//   right   = down × forward               // column 0
// This replaces the old identity rotation (optical axis up, +Z) so that
// the bowl bake reflects a realistic AVM layout where each side camera
// looks sideways at the car, not straight up.
inline std::vector<CameraExtrinsics> MakeSideCameras(
    const std::vector<double>& azimuths_deg) {
    std::vector<CameraExtrinsics> cams;
    for (double az : azimuths_deg) {
        double rad = az * M_PI / 180.0;
        cv::Vec3d pos(std::cos(rad) * 3.0, std::sin(rad) * 3.0, 1.0);
        cv::Vec3d forward(std::cos(rad), std::sin(rad), 0.0);
        cv::Vec3d down(0.0, 0.0, -1.0);
        cv::Vec3d right = down.cross(forward);
        cv::Matx33d R(right[0], down[0], forward[0],
                      right[1], down[1], forward[1],
                      right[2], down[2], forward[2]);
        cams.push_back({R, pos});
    }
    return cams;
}

// Construct a coarse bowl and bake it for the given cameras in one step.
// r_outter is the only knob; the rest comes from the derived reference-car
// bowl (DeriveBowlParams at the car anchor), with a coarse grid for speed.
inline BowlParams MakeCoarseBowlParams(double r_outter) {
    BowlParams bowl = DeriveBowlParams(5.6, 2.2, 2.9);
    bowl.car_center = cv::Vec2d(0.0, 0.0);
    bowl.step = 1.0;        // coarse grid for test speed
    bowl.r_outter = r_outter;
    return bowl;
}

inline BowlModel MakeCoarseBowl(double r_outter,
                                const std::vector<CameraIntrinsics>& intrs,
                                const std::vector<CameraExtrinsics>& extrs) {
    return BowlModel(MakeCoarseBowlParams(r_outter), intrs, extrs);
}

// Cameras mounted on a ring that LOOK AT the ground around the car (realistic
// AVM setup): the flat bowl centre is visible, so RenderFrame output is not
// black. Camera at (r*cos az, r*sin az, h) looking toward (0, 0, target_z).
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
    cv::Vec3d down = -u;   // camera y axis points DOWN (OpenCV convention)
    // T_wc convention: columns = [right | down | forward], y axis down per OpenCV.
    cv::Matx33d R(right[0], down[0], f[0],
                  right[1], down[1], f[1],
                  right[2], down[2], f[2]);
    return {R, pos};
}

// N look-at cameras evenly spaced around the car.
inline std::vector<CameraExtrinsics> MakeLookAtCameras(int num_cameras) {
    std::vector<CameraExtrinsics> cams;
    for (int i = 0; i < num_cameras; ++i) {
        cams.push_back(MakeLookAtCamera(i * 360.0 / num_cameras, 4.0, 2.0, 0.0));
    }
    return cams;
}

// Build the OSG scene for a coarse bowl baked for N side cameras.
// Returns the scene (root + texture upload interface).
struct TestScene {
    std::unique_ptr<StitchScene> scene;
    bool has_visible_vertices = false;
};

// The camera texture array bound in the bowl stateset (observed the same way
// the engine used to: texture unit 0 attribute of the bowl stateset).
inline osg::ref_ptr<osg::Texture2DArray> SceneTexture(const StitchScene& scene) {
    return dynamic_cast<osg::Texture2DArray*>(
        scene.bowl_stateset()->getTextureAttribute(0, osg::StateAttribute::TEXTURE));
}

inline TestScene BuildTestSceneWithExtrinsics(
    int num_cameras, const std::vector<CameraExtrinsics>& extrs) {
    TestScene ts;
    std::vector<CameraIntrinsics> intrs(num_cameras, MakeIntrinsics());
    auto model = MakeCoarseBowl(12.6, intrs, extrs);

    ts.has_visible_vertices = model.HasVisibleVertices();

    ts.scene = std::make_unique<StitchScene>(model, intrs[0].width, intrs[0].height);
    return ts;
}

inline TestScene BuildTestScene(int num_cameras) {
    return BuildTestSceneWithExtrinsics(num_cameras, MakeLookAtCameras(num_cameras));
}

// Synthetic BGR frames, one distinct colour per camera (BGR order).
// Size matches MakeIntrinsics() so texture upload has no dimension mismatch.
inline std::vector<cv::Mat> MakeColourFrames(int num_cameras) {
    const cv::Scalar colours[4] = {
        cv::Scalar(0, 0, 255),     // camera 0: red
        cv::Scalar(0, 255, 0),     // camera 1: green
        cv::Scalar(255, 0, 0),     // camera 2: blue
        cv::Scalar(0, 255, 255),   // camera 3: yellow
    };
    CameraIntrinsics intr = MakeIntrinsics();
    std::vector<cv::Mat> frames;
    for (int i = 0; i < num_cameras; ++i) {
        frames.push_back(cv::Mat(intr.height, intr.width, CV_8UC3, colours[i % 4]));
    }
    return frames;
}

// Count vivid (high-saturation) pixels — the camera-colour regions of a view.
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

} // namespace avm_stitch::test
