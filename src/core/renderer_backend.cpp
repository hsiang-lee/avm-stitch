#include "core/renderer_backend.hpp"

#include "common/base64.h"
#include "common/image_converter.hpp"
#include "core/calib_files_generator.hpp"
#include "core/stitch_result.hpp"
#include "core/view_mapping.hpp"
#include "render/egl_gles3_context.hpp"
#include "render/fbo_manager.hpp"

#include <GLES3/gl3.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace avm_worker { namespace avm_stitch {

namespace {

constexpr int kOutputWidth = 960;
constexpr int kOutputHeight = 640;

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

// Case-insensitive ASCII string comparison.
bool iequals(const std::string& a, const std::string& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::tolower(static_cast<unsigned char>(a[i])) !=
            std::tolower(static_cast<unsigned char>(b[i]))) {
            return false;
        }
    }
    return true;
}

// Match the calibration camera whose id or label equals `role`
// (left/right/front/rear, case-insensitive) and return the key under which
// its image lives in the task's `images` object. Returns "" when no match.
std::string FindImageKeyForRole(const nlohmann::json& images,
                                const nlohmann::json& calibration,
                                const std::string& role) {
    if (!calibration.contains("cameras") ||
        !calibration["cameras"].is_array()) {
        return "";
    }
    for (const auto& cam : calibration["cameras"]) {
        std::string id = cam.contains("id") ? cam["id"].get<std::string>() : "";
        std::string label =
            cam.contains("label") ? cam["label"].get<std::string>() : "";
        if (iequals(id, role) || iequals(label, role)) {
            for (auto it = images.begin(); it != images.end(); ++it) {
                if (iequals(it.key(), id) || iequals(it.key(), label)) {
                    return it.key();
                }
            }
        }
    }
    return "";
}

// Decode the 4 camera images in [left, right, front, rear] order (the
// AVM_INPUT_FRAME layout) into `storage` and return non-owning RGBA pointers.
// Throws if a camera image is missing or the decoded sizes disagree.
std::vector<const std::uint8_t*> DecodeRoleBuffers(
        const nlohmann::json& images,
        const nlohmann::json& calibration, int& w, int& h,
        std::array<DecodedImage, 4>& storage) {
    static const char* roles[4] = {"left", "right", "front", "rear"};
    for (int i = 0; i < 4; ++i) {
        std::string key = FindImageKeyForRole(images, calibration, roles[i]);
        if (key.empty()) {
            throw std::runtime_error(
                "Missing image for camera: " + std::string(roles[i]));
        }
        storage[i] = DecodeBase64JpegToRgba(images[key].get<std::string>());
    }
    // All four cameras must produce identical frame sizes (avm-renderer
    // assumes a single input resolution).
    for (int i = 1; i < 4; ++i) {
        if (storage[i].width != storage[0].width ||
            storage[i].height != storage[0].height) {
            throw std::runtime_error("Camera image size mismatch");
        }
    }
    w = storage[0].width;
    h = storage[0].height;
    std::vector<const std::uint8_t*> ptrs;
    ptrs.reserve(4);
    for (const auto& d : storage) {
        ptrs.push_back(d.rgba_data.data());
    }
    return ptrs;
}

// Run model_creator for the three model kinds (bowl_boat / aerial / distort).
// Throws on a non-zero exit code.
void RunModelCreator(const std::string& tmp_base) {
    std::string mc_path = std::getenv("MODEL_CREATOR_PATH")
        ? std::getenv("MODEL_CREATOR_PATH") : "tools/model_creator";
    std::string mc_cmd = "cd " + tmp_base + " && " + mc_path +
        " --create bowl_boat && " + mc_path + " --create aerial && " + mc_path +
        " --create distort 2>>" + tmp_base + "/stderr_mc.log";
    if (std::system(mc_cmd.c_str()) != 0) {
        throw std::runtime_error("model_creator exited with non-zero code");
    }
}

// Copy RESOURCES_DIR into tmp_base/resources and strip .template files.
void CopyResources(const std::string& tmp_base) {
    const char* res_dir = std::getenv("RESOURCES_DIR");
    if (!res_dir) return;
    std::filesystem::copy(res_dir, tmp_base + "/resources",
        std::filesystem::copy_options::recursive |
        std::filesystem::copy_options::overwrite_existing);
    for (auto& p : std::filesystem::recursive_directory_iterator(
             tmp_base + "/resources")) {
        if (p.path().extension() == ".template") {
            std::filesystem::remove(p.path());
        }
    }
}

// Read back the FBO color attachment, flip vertically (OpenGL bottom-left
// origin → PNG top-left origin), convert RGBA→BGR, encode as PNG, and
// base64-encode it for the result envelope.
std::string ReadbackViewAsBase64Png(FboManager& fbo, int w, int h) {
    auto rgba = fbo.GlReadPixels(w, h, GL_RGBA);  // GLES3 only supports GL_RGBA
    cv::Mat mat(h, w, CV_8UC4, rgba.data());
    cv::Mat flipped;
    cv::flip(mat, flipped, 0);
    cv::Mat bgr;
    cv::cvtColor(flipped, bgr, cv::COLOR_RGBA2BGR);
    std::vector<std::uint8_t> png;
    if (!cv::imencode(".png", bgr, png)) {
        throw std::runtime_error("Failed to encode view PNG");
    }
    return base64_encode(png);
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

    // Validate task_id before it is used to build shell commands or
    // filesystem paths (task JSON may come from an external source).
    const auto is_safe_task_id = [](const std::string& s) {
        if (s.empty() || s.size() > 64) return false;
        return std::all_of(s.begin(), s.end(), [](unsigned char c) {
            return std::isalnum(c) || c == '_' || c == '-';
        });
    };
    if (!is_safe_task_id(task_id)) {
        return MakeStitchError(task_id, project_id, "Invalid task_id");
    }

    // Extract images / calibration without throwing on non-object tasks;
    // the "Task must be a JSON object" guard below then reports the error.
    nlohmann::json images = nlohmann::json::object();
    nlohmann::json calibration = nlohmann::json::object();
    if (task.is_object()) {
        images = task.value("images", nlohmann::json::object());
        calibration = task.value("calibration", nlohmann::json::object());
    }

    // Input validation first (same guards as the subprocess/OSG backends):
    // a malformed task is reported before we even touch the renderer.
    try {
        if (!task.is_object()) {
            throw std::runtime_error("Task must be a JSON object");
        }
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

    // Blocking point A fix: the EGL GLES3 context is owned by Run() as a local
    // variable (not a member of Impl), and renderer->Destroy() is called
    // before the function returns so GL resources are released while the
    // context is still current.
    EglGles3Context egl;
    nlohmann::json result;
    try {
        const std::string tmp_base = "/tmp/stitch/" + task_id;
        std::filesystem::create_directories(tmp_base + "/calib/data");
        std::filesystem::create_directories(tmp_base + "/resources/config");

        // 4. Decode the 4 camera images BEFORE model_creator so a missing or
        //    corrupt image fails fast without depending on model_creator.
        std::array<DecodedImage, 4> imgs;
        int iw = 0, ih = 0;
        auto buffers = DecodeRoleBuffers(images, calibration, iw, ih, imgs);

        // 5. Calibration files (template dir: TEMPLATES_DIR or "tools").
        const char* td = std::getenv("TEMPLATES_DIR");
        std::string templates = td ? td : "tools";
        CalibFilesGenerator(templates).Generate(calibration, tmp_base);

        // 6. model_creator subprocess (throws on failure).
        RunModelCreator(tmp_base);

        // 7. Copy resources (strips .template files).
        CopyResources(tmp_base);

        // 8. Explicit EGL GLES3 context + 9. FBO (current context exists now).
        if (!egl.Init() || !egl.MakeCurrent()) {
            throw std::runtime_error("EGL GLES3 context init failed");
        }
        FboManager fbo;
        if (!fbo.Init(kOutputWidth, kOutputHeight)) {
            throw std::runtime_error("FBO init failed");
        }

        // 10. renderer->Init (RealRenderer assumes a current GLES3 context).
        if (!impl_->renderer->Init(tmp_base + "/calib", fbo.GetFboId(),
                                   kOutputWidth, kOutputHeight, 0)) {
            throw std::runtime_error("avm-renderer init failed");
        }

        // 11. Serial 5-view render loop.
        nlohmann::json images_obj = nlohmann::json::object();
        nlohmann::json view_errors = nlohmann::json::object();
        auto start = std::chrono::steady_clock::now();
        for (const char* view : kRenderViewNames) {
            if (!impl_->renderer->UpdateView(MapViewNameToDisplayMode(view))) {
                view_errors[view] = "UpdateView failed";
                continue;
            }
            if (!impl_->renderer->UpdateFrame(buffers, iw, ih, 0)) {
                view_errors[view] = "UpdateFrame failed";
                continue;
            }
            if (!impl_->renderer->Render()) {
                view_errors[view] = "Render failed";
                continue;
            }
            images_obj[view] = ReadbackViewAsBase64Png(fbo, kOutputWidth,
                                                       kOutputHeight);
        }
        // 12. All views failed → stitch_error (same contract as the
        //     subprocess/OSG backends).
        if (images_obj.empty()) {
            // Destroy the renderer before returning so GL resources are
            // released while the EGL context is still current.
            impl_->renderer->Destroy();
            return MakeStitchError(task_id, project_id, "All views failed");
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();

        // 13. Build the standard result envelope.
        result = BuildStitchResult(task_id, project_id, images_obj,
                                   view_errors, elapsed);
    } catch (const std::exception& e) {
        result = MakeStitchError(task_id, project_id, e.what());
    }

    // 14. Destroy the renderer before the local EGL context goes away (egl is
    //     a Run() local; its destructor runs after renderer->Destroy()).
    impl_->renderer->Destroy();
    return result;
}

}} // namespace avm_worker::avm_stitch
