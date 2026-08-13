#include "core/renderer_backend.hpp"

#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <utility>

namespace avm_worker { namespace avm_stitch {

namespace {

// Build the standard stitch_error JSON envelope (same contract as the
// subprocess/OSG backends).
nlohmann::json MakeStitchError(const std::string& task_id,
                               const std::string& project_id,
                               const std::string& message) {
    nlohmann::json err;
    err["type"] = "stitch_error";
    err["task_id"] = task_id;
    err["project_id"] = project_id;
    err["error"] = message;
    return err;
}

}  // namespace

// ── pImpl ──
class RendererBackend::Impl {
public:
    std::unique_ptr<RendererInterface> renderer;

    explicit Impl(std::unique_ptr<RendererInterface> r)
        : renderer(std::move(r)) {}
};

RendererBackend::RendererBackend(std::unique_ptr<RendererInterface> renderer)
    : impl_(std::make_unique<Impl>(std::move(renderer))) {}

RendererBackend::~RendererBackend() = default;

nlohmann::json RendererBackend::Run(const nlohmann::json& task) {
    // Extract task_id / project_id safely.
    std::string task_id = "unknown";
    std::string project_id = "unknown";
    try {
        if (task.is_object()) {
            task_id = task.value("task_id", "unknown");
            project_id = task.value("project_id", "unknown");
        }
    } catch (...) {}

    // Input validation first (same guards as the subprocess/OSG backends):
    // a malformed task is reported before we even touch the renderer.
    try {
        if (!task.is_object()) {
            throw std::runtime_error("Task must be a JSON object");
        }
        auto images = task.value("images", nlohmann::json::object());
        auto calibration = task.value("calibration", nlohmann::json::object());
        if (images.empty()) {
            throw std::runtime_error("Missing 'images' in task");
        }
        if (calibration.empty()) {
            throw std::runtime_error("Missing 'calibration' in task");
        }
    } catch (const std::exception& e) {
        return MakeStitchError(task_id, project_id, e.what());
    }

    // Degraded mode: no avm-renderer available.
    if (!impl_->renderer) {
        return MakeStitchError(task_id, project_id,
                               "avm-renderer library not available");
    }

    // Placeholder: full rendering pipeline implemented in Task 6.
    return MakeStitchError(task_id, project_id,
                           "RendererBackend not yet fully implemented");
}

}} // namespace avm_worker::avm_stitch
