#include "core/stitch_backend.hpp"

#include "core/osg_backend.hpp"
#include "core/renderer_backend.hpp"

#ifdef AVMRENDERER_AVAILABLE
#include "core/real_renderer.hpp"
#endif

namespace avm_worker { namespace avm_stitch {

std::unique_ptr<StitchBackend> CreateStitchBackend(const std::string& name) {
    if (name == "osg") {
        return std::make_unique<OSGBackend>();
    }
    if (name == "renderer") {
#ifdef AVMRENDERER_AVAILABLE
        return std::make_unique<RendererBackend>(
            std::make_unique<RealRenderer>());
#else
        return std::make_unique<RendererBackend>(nullptr);
#endif
    }
    return nullptr;
}

}} // namespace avm_worker::avm_stitch
