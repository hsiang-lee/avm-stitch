#pragma once
#include <nlohmann/json.hpp>
#include <memory>
#include <functional>
#include <string>

namespace avm_worker { namespace avm_stitch {

// Unified stitch backend interface. Both algorithms implement Run(task)->json.
class StitchBackend {
public:
    virtual ~StitchBackend() = default;
    // task: full task JSON (images, calibration, bowl, algorithm, ...)
    // returns: stitch_result JSON (same contract as BuildStitchResult) or stitch_error JSON
    virtual nlohmann::json Run(const nlohmann::json& task) = 0;
};

// Executor injected by the host (avm-worker) for the subprocess pipeline.
// Keeps this library free of the Redis/hiredis task layer.
using StitchExecutor = std::function<nlohmann::json(const nlohmann::json&)>;

// Factory: task["algorithm"] == "osg" -> OSGBackend; else -> SubprocessBackend.
// executor is required for the subprocess backend (nullptr -> throws).
std::unique_ptr<StitchBackend> CreateStitchBackend(
    const std::string& name, StitchExecutor executor = nullptr);

}} // namespace avm_worker::avm_stitch
