#include "core/subprocess_backend.hpp"

#include <stdexcept>
#include <utility>

namespace avm_worker { namespace avm_stitch {

SubprocessBackend::SubprocessBackend(StitchExecutor executor)
    : executor_(std::move(executor)) {
    if (!executor_) {
        throw std::invalid_argument(
            "SubprocessBackend requires a StitchExecutor (subprocess pipeline)");
    }
}

nlohmann::json SubprocessBackend::Run(const nlohmann::json& task) {
    // The subprocess pipeline is provided by the host (avm-worker's
    // StitchEval); this library only holds the injected executor.
    return executor_(task);
}

}} // namespace avm_worker::avm_stitch
