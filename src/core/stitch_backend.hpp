#pragma once
#include <nlohmann/json.hpp>
#include <memory>
#include <string>

namespace avm_stitch {

// Stitch backend interface: Run(task) -> json. The OSG engine implements it;
// the renderer proxy (migrated to avm-worker) implements the same contract.
class StitchBackend {
public:
    virtual ~StitchBackend() = default;
    // task: full task JSON (images, calibration, bowl, algorithm, ...)
    // returns: stitch_result JSON (same contract as BuildStitchResult) or stitch_error JSON
    virtual nlohmann::json Run(const nlohmann::json& task) = 0;
};

// Factory: task["algorithm"] == "osg" -> OSGBackend; any other name
// (including "renderer", whose proxy moved to avm-worker) -> nullptr.
std::unique_ptr<StitchBackend> CreateStitchBackend(const std::string& name);

} // namespace avm_stitch
