#include "core/renderer_helpers.hpp"

#include "common/base64.h"
#include "render/fbo_manager.hpp"

#include <GLES3/gl3.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace avm_worker { namespace avm_stitch {

// The AVM_INPUT_FRAME camera layout used by the decode/save helpers.
static const char* kRoleNames[4] = {"left", "right", "front", "rear"};

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
    for (int i = 0; i < 4; ++i) {
        std::string key = FindImageKeyForRole(images, calibration, kRoleNames[i]);
        if (key.empty()) {
            throw std::runtime_error(
                "Missing image for camera: " + std::string(kRoleNames[i]));
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

// Save the 4 decoded fisheye images as PNGs under tmp_base for debugging.
// Best-effort: write failures go to stderr and do not throw.
void SaveDecodedRoleImages(const std::array<DecodedImage, 4>& imgs,
                           const std::string& tmp_base) {
    for (int i = 0; i < 4; ++i) {
        const auto& d = imgs[i];
        if (d.rgba_data.empty() || d.width <= 0 || d.height <= 0) continue;
        cv::Mat rgba(d.height, d.width, CV_8UC4,
                     const_cast<std::uint8_t*>(d.rgba_data.data()));
        cv::Mat bgr;
        cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
        const std::string path = tmp_base + "/" + kRoleNames[i] + ".png";
        if (!cv::imwrite(path, bgr)) {
            std::cerr << "[avm-stitch] Failed to write " << path << std::endl;
        }
    }
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

}} // namespace avm_worker::avm_stitch
