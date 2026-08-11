#include "avm_stitch/core/stitch_engine.hpp"

#include "avm_stitch/geometry/blend_weights.hpp"
#include "avm_stitch/geometry/mesh_bake.hpp"
#include "avm_stitch/render/osg_scene.hpp"
#include "avm_stitch/render/osg_view.hpp"
#include "avm_stitch/util/view_config.hpp"

#include <osg/Geode>
#include <osg/StateSet>
#include <osg/Texture2DArray>

#include <stdexcept>
#include <string>

namespace avm_worker { namespace avm_stitch {

namespace {

// The camera texture array is created inside BuildOSGScene and bound in the
// bowl geode's stateset; RenderFrame must upload into exactly that texture.
osg::Texture2DArray* FindSceneTexture(osg::Group* root) {
    if (!root) return nullptr;
    for (unsigned int i = 0; i < root->getNumChildren(); ++i) {
        if (auto* geode = dynamic_cast<osg::Geode*>(root->getChild(i))) {
            if (auto* ss = geode->getStateSet()) {
                return dynamic_cast<osg::Texture2DArray*>(
                    ss->getTextureAttribute(0, osg::StateAttribute::TEXTURE));
            }
        }
    }
    return nullptr;
}

} // namespace

struct StitchEngine::Impl {
    osg::ref_ptr<osg::Group> root;
    osg::ref_ptr<osg::Texture2DArray> texture_array;
    std::unique_ptr<OSGViewRenderer> renderer;
};

StitchEngine::StitchEngine() : impl_(std::make_unique<Impl>()) {}

StitchEngine::~StitchEngine() = default;

void StitchEngine::Configure(
    const std::vector<CameraIntrinsics>& intrinsics,
    const std::vector<CameraExtrinsics>& extrinsics,
    const BowlParams& bowl_params,
    int output_width, int output_height) {
    const int N = static_cast<int>(intrinsics.size());
    if (N == 0 || N > 8) {
        throw std::invalid_argument(
            "StitchEngine::Configure: N must be in [1, 8], got " + std::to_string(N));
    }
    if (extrinsics.size() != static_cast<size_t>(N)) {
        throw std::invalid_argument(
            "StitchEngine::Configure: intrinsics and extrinsics size mismatch");
    }
    // The camera texture array is one shared 2D array sized from intrinsics[0];
    // every camera image must match it, so reject heterogeneous image sizes.
    for (int i = 1; i < N; ++i) {
        if (intrinsics[i].width != intrinsics[0].width ||
            intrinsics[i].height != intrinsics[0].height) {
            throw std::invalid_argument(
                "StitchEngine::Configure: all cameras must share the same image "
                "size; camera 0 is " + std::to_string(intrinsics[0].width) + "x" +
                std::to_string(intrinsics[0].height) + ", camera " + std::to_string(i) +
                " is " + std::to_string(intrinsics[i].width) + "x" +
                std::to_string(intrinsics[i].height));
        }
    }
    // Degenerate bowl params would produce an empty or exploding mesh grid
    // (rows = r_outter / step): reject them with a clear error instead of UB.
    if (bowl_params.step <= 0.0) {
        throw std::invalid_argument(
            "StitchEngine::Configure: bowl_params.step must be > 0, got " +
            std::to_string(bowl_params.step));
    }
    if (bowl_params.r_outter <= 0.0) {
        throw std::invalid_argument(
            "StitchEngine::Configure: bowl_params.r_outter must be > 0, got " +
            std::to_string(bowl_params.r_outter));
    }
    if (bowl_params.k_slop < 0.0) {
        throw std::invalid_argument(
            "StitchEngine::Configure: bowl_params.k_slop must be >= 0, got " +
            std::to_string(bowl_params.k_slop));
    }
    if (bowl_params.z_exp < 0.0) {
        throw std::invalid_argument(
            "StitchEngine::Configure: bowl_params.z_exp must be >= 0, got " +
            std::to_string(bowl_params.z_exp));
    }

    // Phase 1 pipeline: bowl mesh -> bake UV/weights -> OSG scene graph.
    const BowlMesh mesh = CreateBowlMesh(bowl_params);
    const BakedMesh baked = BakeMesh(mesh, intrinsics, extrinsics);

    const int img_w = intrinsics[0].width;
    const int img_h = intrinsics[0].height;
    impl_->root = BuildOSGScene(mesh, baked, N, img_w, img_h);
    // Upload into the texture that the scene's stateset actually binds.
    impl_->texture_array = FindSceneTexture(impl_->root.get());

    impl_->renderer = std::make_unique<OSGViewRenderer>();
    // Viewer realized lazily; the EGL headless GL context is created eagerly in Initialize().
    impl_->renderer->Initialize(impl_->root, DefaultViewPoses(),
                                output_width, output_height);

    num_cameras_ = N;
    configured_ = true;
}

std::map<std::string, cv::Mat> StitchEngine::RenderFrame(
    const std::vector<cv::Mat>& images) const {
    if (!configured_) {
        throw std::runtime_error("StitchEngine::RenderFrame called before Configure()");
    }
    if (images.size() != static_cast<size_t>(num_cameras_)) {
        throw std::invalid_argument(
            "StitchEngine::RenderFrame: expected " + std::to_string(num_cameras_) +
            " images, got " + std::to_string(images.size()));
    }

    UploadCameraImages(impl_->texture_array.get(), images);
    return impl_->renderer->RenderAllViews();
}

bool StitchEngine::IsConfigured() const { return configured_; }

int StitchEngine::GetNumCameras() const { return num_cameras_; }

}} // namespace avm_worker::avm_stitch
