#include "core/stitch_engine.hpp"

#include "geometry/blend_weights.hpp"
#include "geometry/mesh_bake.hpp"
#include "render/osg_scene.hpp"
#include "render/osg_view.hpp"
#include "util/view_config.hpp"

#include <osg/Geode>
#include <osg/StateSet>
#include <osg/Texture2DArray>

#include <stdexcept>
#include <string>

namespace avm_stitch {

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
    bool configured = false;
    int num_cameras = 0;

    /// Validate Configure arguments (N range, size match, shared image size,
    /// bowl params). Throws std::invalid_argument on the first violation.
    void ValidateConfigureArgs(
        const std::vector<CameraIntrinsics>& intrinsics,
        const std::vector<CameraExtrinsics>& extrinsics,
        const BowlParams& bowl_params) const {
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
    }

    void Configure(const std::vector<CameraIntrinsics>& intrinsics,
                   const std::vector<CameraExtrinsics>& extrinsics,
                   const BowlParams& bowl_params,
                   int output_width, int output_height) {
        ValidateConfigureArgs(intrinsics, extrinsics, bowl_params);
        const int N = static_cast<int>(intrinsics.size());

        // Phase 1 pipeline: bowl model -> bake UV/weights -> OSG scene graph.
        const BowlModel model(bowl_params);
        const BakedMesh baked(model, intrinsics, extrinsics);

        const int img_w = intrinsics[0].width;
        const int img_h = intrinsics[0].height;
        root = BuildOSGScene(model, baked, N, img_w, img_h);
        // Upload into the texture that the scene's stateset actually binds.
        texture_array = FindSceneTexture(root.get());

        renderer = std::make_unique<OSGViewRenderer>();
        // Viewer realized lazily; the EGL headless GL context is created eagerly in Initialize().
        renderer->Initialize(root, DefaultViewPoses(), output_width, output_height);

        num_cameras = N;
        configured = true;
    }

    std::map<std::string, cv::Mat> RenderFrame(
        const std::vector<cv::Mat>& images) const {
        if (!configured) {
            throw std::runtime_error("StitchEngine::RenderFrame called before Configure()");
        }
        if (images.size() != static_cast<size_t>(num_cameras)) {
            throw std::invalid_argument(
                "StitchEngine::RenderFrame: expected " + std::to_string(num_cameras) +
                " images, got " + std::to_string(images.size()));
        }

        UploadCameraImages(texture_array.get(), images);
        return renderer->RenderAllViews();
    }

    bool IsConfigured() const { return configured; }

    int GetNumCameras() const { return num_cameras; }
};

StitchEngine::StitchEngine() : impl_(std::make_unique<Impl>()) {}

StitchEngine::~StitchEngine() = default;

void StitchEngine::Configure(const std::vector<CameraIntrinsics>& intrinsics,
                             const std::vector<CameraExtrinsics>& extrinsics,
                             const BowlParams& bowl_params,
                             int output_width, int output_height) {
    impl_->Configure(intrinsics, extrinsics, bowl_params,
                     output_width, output_height);
}

std::map<std::string, cv::Mat> StitchEngine::RenderFrame(
    const std::vector<cv::Mat>& images) const {
    return impl_->RenderFrame(images);
}

bool StitchEngine::IsConfigured() const { return impl_->IsConfigured(); }

int StitchEngine::GetNumCameras() const { return impl_->GetNumCameras(); }

} // namespace avm_stitch
