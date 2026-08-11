#pragma once

#include "avm_stitch/stitch_backend.hpp"

#include <functional>
#include <stdexcept>

namespace avm_worker { namespace avm_stitch {

// Existing subprocess pipeline (model_creator + stitcher) as a backend.
// Delegates to an externally injected StitchExecutor callback.
class SubprocessBackend : public StitchBackend {
public:
    explicit SubprocessBackend(StitchExecutor executor);
    nlohmann::json Run(const nlohmann::json& task) override;
private:
    StitchExecutor executor_;
};

}} // namespace avm_worker::avm_stitch
