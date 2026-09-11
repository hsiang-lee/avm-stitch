#include "render/osg_view_renderer.hpp"

#include "render/egl_headless_context.hpp"

#include <osg/Camera>
#include <osg/GraphicsContext>
#include <osg/Drawable>
#include <osg/Geode>
#include <osg/StateSet>
#include <osg/State>
#include <osg/Uniform>

#include <osgGA/TrackballManipulator>

#include <opencv2/imgproc.hpp>

#include <cstdlib>

namespace avm_stitch {

namespace {

constexpr double kNear = 0.1;
constexpr double kFar = 100.0;
constexpr int kWindowSize = 16;   // tiny pbuffer that owns the GL context

// Background clear colour RGB(30,36,50) — matches the stitcher default.
osg::Vec4 BackgroundClearColor() {
    return osg::Vec4(30.0 / 255.0, 36.0 / 255.0, 50.0 / 255.0, 1.0);
}

// VCS -> OpenGL coordinate conversion: (x, y, z) -> (x, z, -y).
osg::Vec3 VcsToOgl(const cv::Vec3d& v) {
    return osg::Vec3(v[0], v[2], -v[1]);
}

// One FBO camera per view pose: renders the bowl geometry with its own
// view/projection into an attached osg::Image (RGBA). Each camera gets its own
// Geode that references the SAME Geometry drawable and StateSet: osg::Node has
// single-parent ownership, so the same Geode cannot be a child of several
// cameras (addChild would silently reparent it to the last one). Sharing the
// Drawable/StateSet across Geodes is legal — only Nodes are single-parent.
osg::ref_ptr<osg::Camera> CreateViewCamera(const ViewPose& pose,
                                           const osg::ref_ptr<osg::Drawable>& bowl_drawable,
                                           const osg::ref_ptr<osg::StateSet>& bowl_stateset,
                                           int width, int height) {
    auto cam = new osg::Camera;

    cam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    cam->setRenderOrder(osg::Camera::PRE_RENDER);
    cam->setReferenceFrame(osg::Transform::ABSOLUTE_RF);

    auto color_image = new osg::Image;
    color_image->allocateImage(width, height, 1, GL_RGBA, GL_UNSIGNED_BYTE);
    cam->attach(osg::Camera::COLOR_BUFFER, color_image);

    cam->setViewport(0, 0, width, height);
    cam->setClearColor(BackgroundClearColor());
    cam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    double aspect = static_cast<double>(width) / static_cast<double>(height);
    cam->setProjectionMatrixAsPerspective(pose.fovy_deg, aspect, kNear, kFar);
    cam->setViewMatrixAsLookAt(VcsToOgl(pose.position), VcsToOgl(pose.target),
                               VcsToOgl(pose.up));

    // The vcpkg osg GL2 build does not auto-update osg_ModelViewProjectionMatrix
    // (fixed-function mode), so supply it explicitly per camera. Must use
    // osg::Matrixf: Matrixd uniforms trigger GL_INVALID_OPERATION on this build.
    osg::Matrixf mvp(cam->getViewMatrix() * cam->getProjectionMatrix());
    cam->getOrCreateStateSet()->addUniform(
        new osg::Uniform("osg_ModelViewProjectionMatrix", mvp));

    if (bowl_drawable) {
        auto geode = new osg::Geode;
        geode->addDrawable(bowl_drawable.get());   // SAME geometry, own geode — legal
        if (bowl_stateset) geode->setStateSet(bowl_stateset.get());
        cam->addChild(geode);
    }
    return cam;
}

// Pre-draw callback that recomputes osg_ModelViewProjectionMatrix from the
// camera's own view/projection matrices every frame. Required in Windowed
// mode: the TrackballManipulator changes the view matrix per frame, but the
// vcpkg OSG GL2 build does not auto-update the built-in uniform. NOTE: must
// read camera_->getViewMatrix(), NOT state->getModelViewMatrix() — the
// latter returns identity when GLSL shaders are active on this build.
struct MVPUpdateCallback : public osg::Camera::DrawCallback {
    osg::ref_ptr<osg::Uniform> uniform_;
    osg::ref_ptr<osg::Camera> camera_;

    MVPUpdateCallback(osg::Uniform* u, osg::Camera* cam) : uniform_(u), camera_(cam) {}

    void operator()(osg::RenderInfo& renderInfo) const override {
        if (!uniform_ || !camera_) return;
        osg::Matrixf mvp(camera_->getViewMatrix() * camera_->getProjectionMatrix());
        uniform_->set(mvp);
    }
};

} // namespace

OSGViewRenderer::OSGViewRenderer(const StitchScene& scene,
                                 const std::vector<ViewPose>& view_poses,
                                 int output_width, int output_height,
                                 RenderMode mode,
                                 const cv::Vec2d& car_center) {
    output_width_ = output_width;
    output_height_ = output_height;
    mode_ = mode;
    car_center_ = car_center;
    view_cameras_.clear();
    view_images_.clear();
    view_names_.clear();
    realized_ = false;

    osg::ref_ptr<osg::Group> root = scene.root();

    viewer_ = new osgViewer::Viewer;
    viewer_->setThreadingModel(osgViewer::Viewer::SingleThreaded);

    if (mode == RenderMode::Windowed) {
        SetupWindowed(root);
        return;  // Windowed mode: skip FBO camera creation
    } else {
        // Headless EGL surfaceless GL context (Offscreen).
        osg::ref_ptr<EGLHeadlessContext> gc =
            CreateEGLHeadlessContext(kWindowSize, kWindowSize);
        osg::Camera* main_cam = viewer_->getCamera();
        main_cam->setGraphicsContext(gc.get());
        main_cam->setViewport(0, 0, kWindowSize, kWindowSize);
    }

    SetupOffscreenCameras(scene, view_poses);
}

void OSGViewRenderer::SetupWindowed(osg::ref_ptr<osg::Group> root) {
    // Real X11 window — requires DISPLAY / WAYLAND_DISPLAY.
    osg::ref_ptr<osg::GraphicsContext::Traits> traits =
        new osg::GraphicsContext::Traits;
    traits->x = 100;
    traits->y = 100;
    traits->width = 1280;
    traits->height = 720;
    traits->windowDecoration = true;
    traits->windowName = "avm_viewer";
    traits->doubleBuffer = true;
    traits->samples = 4;

    osg::ref_ptr<osg::GraphicsContext> gc =
        osg::GraphicsContext::createGraphicsContext(traits.get());
    if (!gc) return;  // bail out gracefully if no display

    osg::Camera* main_cam = viewer_->getCamera();
    main_cam->setGraphicsContext(gc);
    main_cam->setViewport(0, 0, traits->width, traits->height);
    main_cam->setClearColor(BackgroundClearColor());
    main_cam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    double aspect =
        static_cast<double>(traits->width) / traits->height;
    main_cam->setProjectionMatrixAsPerspective(60.0, aspect, kNear, kFar);

    // Offset home camera eye/target by car_center so the view
    // centres on the bowl geometry rather than the data origin.
    // VCS(x,y,z) → OGL(x,z,-y), so OGL offset = (cx, 0, -cy).
    const double cx = car_center_[0];
    const double cy = car_center_[1];
    const osg::Vec3d home_eye(10.0 + cx, -6.0, 6.0 - cy);
    const osg::Vec3d home_target(cx, 0.0, 0.5 - cy);
    const osg::Vec3d home_up(0.0, 0.0, 1.0);

    main_cam->setViewMatrixAsLookAt(home_eye, home_target, home_up);

    // vcpkg OSG GL2 build does not auto-update osg_ModelViewProjectionMatrix;
    // the TrackballManipulator changes the view matrix every frame, so
    // recompute the uniform per-draw via PreDraw callback.
    auto mvp_uniform = new osg::Uniform("osg_ModelViewProjectionMatrix",
                                         osg::Matrixf::identity());
    main_cam->getOrCreateStateSet()->addUniform(mvp_uniform);
    main_cam->setPreDrawCallback(new MVPUpdateCallback(mvp_uniform, main_cam));

    // scene.root() already contains bowl_geode_ — do not add it again.

    // Set TrackballManipulator for mouse interaction
    auto manip = new osgGA::TrackballManipulator;
    manip->setDistance(12.0);
    manip->setTransformation(home_eye, home_target, home_up);
    viewer_->setCameraManipulator(manip);

    viewer_->setSceneData(root.get());
}

void OSGViewRenderer::SetupOffscreenCameras(const StitchScene& scene,
                                             const std::vector<ViewPose>& view_poses) {
    osg::ref_ptr<osg::Group> root = scene.root();

    // The bowl drawable + mapping-shader StateSet are shared across all
    // per-view cameras: take them from the scene, then let each camera build
    // its OWN geode around them.
    osg::ref_ptr<osg::Drawable> bowl_drawable = scene.bowl_drawable();
    osg::ref_ptr<osg::StateSet> bowl_stateset = scene.bowl_stateset();

    for (const auto& pose : view_poses) {
        auto cam = CreateViewCamera(pose, bowl_drawable, bowl_stateset,
                                    output_width_, output_height_);
        view_cameras_.push_back(cam);
        view_images_.push_back(
            cam->getBufferAttachmentMap()[osg::Camera::COLOR_BUFFER]._image);
        view_names_.push_back(pose.name);
        root->addChild(cam);
    }

    if (!viewer_->getSceneData()) {
        viewer_->setSceneData(root.get());
    }
}

void OSGViewRenderer::EnsureRealized() {
    if (realized_ || !viewer_.valid()) return;
    viewer_->realize();
    realized_ = true;
}

std::map<std::string, cv::Mat> OSGViewRenderer::RenderAllViews() {
    EnsureRealized();
    viewer_->frame();

    std::map<std::string, cv::Mat> results;
    for (size_t i = 0; i < view_cameras_.size(); ++i) {
        osg::Image* color_image = view_images_[i].get();
        if (!color_image || !color_image->data()) {
            // No rendered pixels (e.g. no GL context): leave an empty Mat.
            results[view_names_[i]] = cv::Mat();
            continue;
        }
        cv::Mat rgba(output_height_, output_width_, CV_8UC4, color_image->data());
        cv::Mat bgr;
        cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
        // FBO readback flip: GL framebuffer row 0 is the scene BOTTOM (GL
        // window origin is bottom-left), but cv::Mat row 0 is the image TOP.
        // The reference subprocess stitcher flips the readback; without this
        // the rendered view is vertically mirrored.
        cv::flip(bgr, bgr, 0);
        results[view_names_[i]] = bgr.clone();
    }
    return results;
}

void OSGViewRenderer::WaitForWindowClose() {
    if (mode_ == RenderMode::Offscreen || !viewer_.valid()) return;
    EnsureRealized();
    // Block until the viewer's window is closed.
    while (!viewer_->done()) {
        viewer_->frame();
    }
}

} // namespace avm_stitch
