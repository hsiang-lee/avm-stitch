#pragma once
#include <nlohmann/json.hpp>
#include <memory>
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

// Factory: task["algorithm"] == "osg" -> OSGBackend; "renderer" -> RendererBackend;
// any unknown name -> nullptr (the host falls back to the renderer backend).
std::unique_ptr<StitchBackend> CreateStitchBackend(const std::string& name);

}} // namespace avm_worker::avm_stitch
