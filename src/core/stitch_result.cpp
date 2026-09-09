#include "core/stitch_result.hpp"

#include "eval_metrics/eval_metrics.hpp"

namespace avm_stitch {

nlohmann::json BuildStitchResult(
    const std::string& task_id,
    const std::string& project_id,
    const nlohmann::json& images_obj,
    const nlohmann::json& view_errors_obj,
    long elapsed_ms) {

    auto metrics = avm_worker::ComputeStitchMetrics();
    double overall = avm_worker::ComputeOverallScore(metrics);
    metrics["overall"] = overall;

    nlohmann::json result;
    result["type"] = "stitch_result";
    result["task_id"] = task_id;
    result["project_id"] = project_id;
    result["images"] = images_obj;

    auto topdown_it = images_obj.find("topdown");
    result["bev_image_base64"] = (topdown_it != images_obj.end())
        ? topdown_it->get<std::string>() : "";

    if (!view_errors_obj.empty()) {
        result["view_errors"] = view_errors_obj;
    }
    result["metrics"] = metrics;
    result["overall_score"] = overall;
    result["processing_time_ms"] = elapsed_ms;

    return result;
}

} // namespace avm_stitch
