#include "avm_stitch/stitch_backend.hpp"

#include "avm_stitch/osg_backend.hpp"
#include "avm_stitch/subprocess_backend.hpp"

#include <utility>

namespace avm_worker { namespace avm_stitch {

std::unique_ptr<StitchBackend> CreateStitchBackend(
    const std::string& name, StitchExecutor executor) {
    if (name == "osg") {
        return std::make_unique<OSGBackend>();
    }
    if (name == "subprocess") {
        return std::make_unique<SubprocessBackend>(std::move(executor));
    }
    return nullptr;
}

}} // namespace avm_worker::avm_stitch
