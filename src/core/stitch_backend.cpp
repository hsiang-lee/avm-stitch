#include "core/stitch_backend.hpp"

#include "core/osg_backend.hpp"

namespace avm_stitch {

std::unique_ptr<StitchBackend> CreateStitchBackend(const std::string& name) {
    if (name == "osg") {
        return std::make_unique<OSGBackend>();
    }
    return nullptr;
}

} // namespace avm_stitch
