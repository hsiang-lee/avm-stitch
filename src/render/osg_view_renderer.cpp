#include "render/osg_view_renderer.hpp"

#include "render/egl_headless_context.hpp"

#include <osg/Camera>
#include <osg/Drawable>
#include <osg/Geode>
#include <osg/GraphicsContext>
#include <osg/State>
#include <osg/StateSet>
#include <osg/Uniform>

#include <osgGA/TrackballManipulator>

#include <opencv2/imgproc.hpp>

#include <algorithm>

namespace avm_stitch {

namespace {

constexpr int kWindowSize = 16;
osg::Vec4 BackgroundClearColor() {
    return osg::Vec4(30.0 / 255.0, 36.0 / 255.0, 50.0 / 255.0, 1.0);
}

osg::Vec3 VcsToOgl(const cv::Vec3d& v) {
    return osg::Vec3(v[0], v[2], -v[1]);
}

osg::ref_ptr<osg::Camera> CreateViewCamera(const ViewPose& pose,
                                           const osg::ref_ptr<osg::Drawable>& bowl_drawable,
                                           const osg::ref_ptr<osg::StateSet>& bowl_stateset,
                                           int width, int height,
                                           const ClipPlanes& clip) {
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
    cam->setProjectionMatrixAsPerspective(pose.fovy_deg, aspect, clip.near, clip.far);
    cam->setViewMatrixAsLookAt(VcsToOgl(pose.position), VcsToOgl(pose.target),
                               VcsToOgl(pose.up));

    osg::Matrixf mvp(cam->getViewMatrix() * cam->getProjectionMatrix());
    cam->getOrCreateStateSet()->addUniform(
        new osg::Uniform("osg_ModelViewProjectionMatrix", mvp));

    if (bowl_drawable) {
        auto geode = new osg::Geode;
        geode->addDrawable(bowl_drawable.get());
        if (bowl_stateset)
            geode->setStateSet(bowl_stateset.get());
        cam->addChild(geode);
    }
    return cam;
}

struct MVPUpdateCallback : public osg::Camera::DrawCallback {
    osg::ref_ptr<osg::Uniform> uniform_;
    osg::ref_ptr<osg::Camera> camera_;

    MVPUpdateCallback(osg::Uniform* u, osg::Camera* cam) : uniform_(u), camera_(cam) {}

    void operator()(osg::RenderInfo& renderInfo) const override {
        if (!uniform_ || !camera_)
            return;
        osg::Matrixf mvp(camera_->getViewMatrix() * camera_->getProjectionMatrix());
        uniform_->set(mvp);
    }
};

}
OSGViewRenderer::OSGViewRenderer(const StitchScene& scene,
                                 const BowlParams& bowl,
                                 int output_width, int output_height,
                                 RenderMode mode)
    : bowl_(bowl) {
    output_width_ = output_width;
    output_height_ = output_height;
    mode_ = mode;
    view_cameras_.clear();
    view_images_.clear();
    view_names_.clear();
    realized_ = false;

    osg::ref_ptr<osg::Group> root = scene.root();

    viewer_ = new osgViewer::Viewer;
    viewer_->setThreadingModel(osgViewer::Viewer::SingleThreaded);

    if (mode == RenderMode::Windowed) {
        SetupWindowed(root);
        return;
    } else {
        osg::ref_ptr<EGLHeadlessContext> gc =
            CreateEGLHeadlessContext(kWindowSize, kWindowSize);
        osg::Camera* main_cam = viewer_->getCamera();
        main_cam->setGraphicsContext(gc.get());
        main_cam->setViewport(0, 0, kWindowSize, kWindowSize);
    }

    SetupOffscreenCameras(scene);
}

void OSGViewRenderer::SetupWindowed(osg::ref_ptr<osg::Group> root) {
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
    if (!gc)
        return;
    osg::Camera* main_cam = viewer_->getCamera();
    main_cam->setGraphicsContext(gc);
    main_cam->setViewport(0, 0, traits->width, traits->height);
    main_cam->setClearColor(BackgroundClearColor());
    main_cam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    double aspect =
        static_cast<double>(traits->width) / traits->height;

    const ViewPose home = bowl_.HomeViewPose();
    const osg::Vec3d home_eye(home.position[0], home.position[1], home.position[2]);
    const osg::Vec3d home_target(home.target[0], home.target[1], home.target[2]);
    const osg::Vec3d home_up(home.up[0], home.up[1], home.up[2]);

    const ClipPlanes clip = bowl_.ClipPlanesFor((home_eye - home_target).length());
    main_cam->setProjectionMatrixAsPerspective(60.0, aspect, clip.near, clip.far);
    main_cam->setViewMatrixAsLookAt(home_eye, home_target, home_up);

    auto mvp_uniform = new osg::Uniform("osg_ModelViewProjectionMatrix",
                                        osg::Matrixf::identity());
    main_cam->getOrCreateStateSet()->addUniform(mvp_uniform);
    main_cam->setPreDrawCallback(new MVPUpdateCallback(mvp_uniform, main_cam));

    auto manip = new osgGA::TrackballManipulator;
    manip->setDistance(bowl_.HomeViewDistance());
    manip->setTransformation(home_eye, home_target, home_up);
    viewer_->setCameraManipulator(manip);

    viewer_->setSceneData(root.get());
}

void OSGViewRenderer::SetupOffscreenCameras(const StitchScene& scene) {
    osg::ref_ptr<osg::Group> root = scene.root();

    osg::ref_ptr<osg::Drawable> bowl_drawable = scene.bowl_drawable();
    osg::ref_ptr<osg::StateSet> bowl_stateset = scene.bowl_stateset();

    const auto view_poses = bowl_.DefaultViewPoses();

    double max_eye = 0.0;
    for (const auto& pose : view_poses) {
        max_eye = std::max(max_eye, cv::norm(pose.position - pose.target));
    }
    const ClipPlanes clip = bowl_.ClipPlanesFor(max_eye);

    for (const auto& pose : view_poses) {
        auto cam = CreateViewCamera(pose, bowl_drawable, bowl_stateset,
                                    output_width_, output_height_, clip);
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
    if (realized_ || !viewer_.valid())
        return;
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
            results[view_names_[i]] = cv::Mat();
            continue;
        }
        cv::Mat rgba(output_height_, output_width_, CV_8UC4, color_image->data());
        cv::Mat bgr;
        cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
        cv::flip(bgr, bgr, 0);
        results[view_names_[i]] = bgr.clone();
    }
    return results;
}

void OSGViewRenderer::WaitForWindowClose() {
    if (mode_ == RenderMode::Offscreen || !viewer_.valid())
        return;
    EnsureRealized();
    while (!viewer_->done()) {
        viewer_->frame();
    }
}

}
