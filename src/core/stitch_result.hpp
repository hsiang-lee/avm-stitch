#pragma once

#include <nlohmann/json.hpp>
#include <string>

namespace avm_stitch {

// Build the standard stitch_result JSON contract. Shared by the subprocess
// and OSG backends so both produce an identical result envelope.
nlohmann::json BuildStitchResult(
    const std::string& task_id,
    const std::string& project_id,
    const nlohmann::json& images_obj,
    const nlohmann::json& view_errors_obj,
    long elapsed_ms);

} // namespace avm_stitch
