#pragma once

#include "avm_stitch/stitch_backend.hpp"

#include <nlohmann/json.hpp>

namespace avm_worker { namespace avm_stitch {

// In-process OSG stitch engine backend. Decodes the task's images/calibration,
// renders the 5 views via StitchEngine, and returns a stitch_result JSON.
// All failures are mapped to a stitch_error JSON (never throws).
class OSGBackend : public StitchBackend {
public:
    nlohmann::json Run(const nlohmann::json& task) override;
};

}} // namespace avm_worker::avm_stitch
