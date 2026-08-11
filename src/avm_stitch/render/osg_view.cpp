#include "avm_stitch/render/osg_view.hpp"

#include "avm_stitch/render/egl_headless_context.hpp"

#include <osg/Drawable>
#include <osg/Geode>
#include <osg/StateSet>
#include <osg/Uniform>

#include <opencv2/imgproc.hpp>

namespace avm_worker { namespace avm_stitch {

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

// Find the first osg::Geode under `group` (the bowl geode added by BuildOSGScene).
osg::Geode* FindBowlGeode(osg::Group* group) {
    if (!group) return nullptr;
    for (unsigned int i = 0; i < group->getNumChildren(); ++i) {
        if (auto* geode = dynamic_cast<osg::Geode*>(group->getChild(i))) {
            return geode;
        }
        if (auto* child_group = group->getChild(i)->asGroup()) {
            if (auto* found = FindBowlGeode(child_group)) return found;
        }
    }
    return nullptr;
}

// One FBO camera per view pose: renders the bowl geometry with its own
// view/projection into an attached osg::Image (RGBA). Each camera gets its own
// Geode that references the SAME Geometry drawable and StateSet: osg::Node has
// single-parent ownership, so the same Geode cannot be a child of several
// cameras (addChild would silently reparent it to the last one). Sharing the
// Drawable/StateSet across Geodes is legal — only Nodes are single-parent.
osg::ref_ptr<osg::Camera> CreateViewCamera(const ViewPose& pose,
                                           osg::Drawable* bowl_drawable,
                                           osg::StateSet* bowl_stateset,
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
        geode->addDrawable(bowl_drawable);   // SAME geometry, own geode — legal
        if (bowl_stateset) geode->setStateSet(bowl_stateset);
        cam->addChild(geode);
    }
    return cam;
}

} // namespace

void OSGViewRenderer::Initialize(osg::ref_ptr<osg::Group> root,
                                 const std::vector<ViewPose>& view_poses,
                                 int output_width, int output_height) {
    output_width_ = output_width;
    output_height_ = output_height;
    view_cameras_.clear();
    view_images_.clear();
    view_names_.clear();
    realized_ = false;

    viewer_ = new osgViewer::Viewer;
    viewer_->setThreadingModel(osgViewer::Viewer::SingleThreaded);
    // Headless EGL surfaceless GL context (desktop GL 3.3 compatibility
    // profile): the stitch renderer runs in docker/CI with no X server and no
    // DISPLAY, so a real X11 window cannot be created.
    osg::ref_ptr<EGLHeadlessContext> gc =
        CreateEGLHeadlessContext(kWindowSize, kWindowSize);
    osg::Camera* cam = viewer_->getCamera();
    cam->setGraphicsContext(gc.get());
    cam->setViewport(0, 0, kWindowSize, kWindowSize);
    viewer_->setSceneData(root.get());

    // The bowl geode (with its Geometry drawable and mapping-shader StateSet)
    // is shared across all per-view cameras: extract the drawable + stateset
    // once, then let each camera build its OWN geode around them.
    osg::Drawable* bowl_drawable = nullptr;
    osg::StateSet* bowl_stateset = nullptr;
    if (osg::Geode* bowl_geode = FindBowlGeode(root.get())) {
        if (bowl_geode->getNumDrawables() > 0) {
            bowl_drawable = bowl_geode->getDrawable(0);
        }
        bowl_stateset = bowl_geode->getStateSet();
    }

    for (const auto& pose : view_poses) {
        auto cam = CreateViewCamera(pose, bowl_drawable, bowl_stateset,
                                    output_width_, output_height_);
        view_cameras_.push_back(cam);
        view_images_.push_back(cam->getBufferAttachmentMap()[osg::Camera::COLOR_BUFFER]._image);
        view_names_.push_back(pose.name);
        root->addChild(cam);
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

}} // namespace avm_worker::avm_stitch
