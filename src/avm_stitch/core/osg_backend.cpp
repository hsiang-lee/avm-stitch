#include "avm_stitch/osg_backend.hpp"

#include "avm_stitch/bowl_model.hpp"
#include "avm_stitch/camera_name.hpp"
#include "avm_stitch/stitch_engine.hpp"
#include "avm_stitch/stitch_result.hpp"
#include "common/base64.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>

#include <chrono>
#include <cctype>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace avm_worker { namespace avm_stitch {

namespace {

struct ParsedCalibration {
    std::vector<CameraIntrinsics> intrinsics;
    std::vector<CameraExtrinsics> extrinsics;
    nlohmann::json cameras;  // raw calibration.cameras[], in calibration order
};

// Case-insensitive string equality (ASCII), matching the subprocess path's
// helper in stitch_eval.cpp.
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

// Find the task image key that matches the camera's id OR label
// (case-insensitive), mirroring the subprocess path's FindCameraIndex.
// Returns an empty string when no image key matches.
std::string FindCameraImageKey(const nlohmann::json& images,
                               const nlohmann::json& cam) {
    const std::string id = cam.contains("id") && cam["id"].is_string()
        ? cam["id"].get<std::string>() : "";
    const std::string label = cam.contains("label") && cam["label"].is_string()
        ? cam["label"].get<std::string>() : "";
    for (auto it = images.begin(); it != images.end(); ++it) {
        if (!it.value().is_string()) continue;
        const std::string& key = it.key();
        if ((!id.empty() && iequals(key, id)) ||
            (!label.empty() && iequals(key, label))) {
            return key;
        }
    }
    return "";
}

// Parse calibration.cameras[] into intrinsics/extrinsics.
// T_wc convention: copy rotation (row-major 9 doubles) and translation
// (3 doubles) verbatim, no transpose/inverse.
ParsedCalibration ParseCalibration(const nlohmann::json& calibration) {
    if (!calibration.contains("cameras") || calibration["cameras"].empty()) {
        throw std::runtime_error("Empty 'cameras' in calibration");
    }
    const auto& cameras = calibration["cameras"];
    ParsedCalibration parsed;
    parsed.cameras = cameras;
    parsed.intrinsics.reserve(cameras.size());
    parsed.extrinsics.reserve(cameras.size());

    for (size_t i = 0; i < cameras.size(); ++i) {
        const auto& cam = cameras[i];

        CameraIntrinsics intr;
        intr.width = cam["width"].get<int>();
        intr.height = cam["height"].get<int>();
        intr.fx = cam["fx"].get<double>();
        intr.fy = cam["fy"].get<double>();
        intr.cx = cam["cx"].get<double>();
        intr.cy = cam["cy"].get<double>();
        intr.k1 = cam["k1"].get<double>();
        intr.k2 = cam["k2"].get<double>();
        intr.k3 = cam["k3"].get<double>();
        intr.k4 = cam["k4"].get<double>();
        if (cam.contains("fisheye_radius") && cam["fisheye_radius"].is_number()) {
            intr.fisheye_radius = cam["fisheye_radius"].get<double>();
        }

        CameraExtrinsics extr;
        const auto& rot = cam["rotation"];
        for (int r = 0; r < 9; ++r) {
            extr.R.val[r] = rot[r].get<double>();
        }
        const auto& tr = cam["translation"];
        extr.t = cv::Vec3d(tr[0].get<double>(), tr[1].get<double>(),
                           tr[2].get<double>());

        parsed.intrinsics.push_back(intr);
        parsed.extrinsics.push_back(extr);
    }
    return parsed;
}

// Decode a base64 JPEG payload; throws a camera-labelled error when the data
// is empty/invalid or the image cannot be decoded.
cv::Mat DecodeBase64Jpeg(const std::string& b64, const std::string& cam_key) {
    auto jpeg_bytes = base64_decode(b64);
    if (jpeg_bytes.empty()) {
        throw std::runtime_error("Empty or invalid base64 data for camera: " +
                                 cam_key);
    }
    cv::Mat img = cv::imdecode(jpeg_bytes, cv::IMREAD_COLOR);
    if (img.empty()) {
        throw std::runtime_error("Failed to decode image for camera: " +
                                 cam_key);
    }
    return img;
}

// Decode camera images in calibration order, keyed by the camera's id or
// label (case-insensitive) — the same robust resolution as the subprocess
// path. The real API keys images by label, so using the camera id as the
// lookup key (ResolveCameraName's preference) would throw a false
// "Missing image for camera" error.
std::vector<cv::Mat> DecodeCameraImages(const nlohmann::json& images,
                                        const ParsedCalibration& parsed) {
    std::vector<cv::Mat> mats;
    mats.reserve(parsed.intrinsics.size());
    for (size_t i = 0; i < parsed.intrinsics.size(); ++i) {
        const auto& cam = parsed.cameras[i];
        std::string key = FindCameraImageKey(images, cam);
        if (key.empty()) {
            // Clear error: report the camera's id if present else its label.
            throw std::runtime_error("Missing image for camera: " +
                                     ResolveCameraName(cam, i));
        }
        mats.push_back(
            DecodeBase64Jpeg(images.at(key).get<std::string>(), key));
    }
    return mats;
}

// Encode each rendered view as a base64 PNG, keyed by view name.
nlohmann::json EncodeViewsAsBase64(
    const std::map<std::string, cv::Mat>& views) {
    nlohmann::json images_obj = nlohmann::json::object();
    for (const auto& [view_name, mat] : views) {
        std::vector<uint8_t> png_buf;
        if (!cv::imencode(".png", mat, png_buf)) {
            throw std::runtime_error("Failed to encode view: " + view_name);
        }
        images_obj[view_name] = base64_encode(png_buf);
    }
    return images_obj;
}

}  // namespace

nlohmann::json OSGBackend::Run(const nlohmann::json& task) {
    try {
        const std::string task_id = task.value("task_id", "unknown");
        const std::string project_id = task.value("project_id", "unknown");

        // 1. Extract images and calibration (same guards as the subprocess path).
        auto images = task.value("images", nlohmann::json::object());
        auto calibration = task.value("calibration", nlohmann::json::object());
        if (images.empty()) {
            throw std::runtime_error("Missing 'images' in task");
        }
        if (calibration.empty()) {
            throw std::runtime_error("Missing 'calibration' in task");
        }

        // 2. Parse calibration.cameras[] into intrinsics/extrinsics.
        ParsedCalibration parsed = ParseCalibration(calibration);

        // 3. Decode camera images in calibration order, keyed by resolved name.
        std::vector<cv::Mat> mats = DecodeCameraImages(images, parsed);

        // 4. Configure the in-process OSG engine and render the 5 views.
        BowlParams bowl = BowlParams::FromJson(task);
        StitchEngine engine;
        engine.Configure(parsed.intrinsics, parsed.extrinsics, bowl, 960, 640);

        auto render_start = std::chrono::steady_clock::now();
        auto views = engine.RenderFrame(mats);

        // 5. Encode each view as base64 PNG.
        nlohmann::json images_obj = EncodeViewsAsBase64(views);
        auto render_end = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            render_end - render_start).count();

        // 6. Build the standard stitch_result contract.
        nlohmann::json view_errors_obj = nlohmann::json::object();
        return BuildStitchResult(task_id, project_id, images_obj,
                                 view_errors_obj, elapsed_ms);

    } catch (const std::exception& e) {
        // Preserve the two-tier error contract: thrown errors become
        // stitch_error JSON at the facade. Never rethrow.
        std::string task_id = "unknown";
        std::string project_id = "unknown";
        try {
            if (task.is_object()) {
                task_id = task.value("task_id", "unknown");
                project_id = task.value("project_id", "unknown");
            }
        } catch (...) {
        }
        nlohmann::json err;
        err["type"] = "stitch_error";
        err["task_id"] = task_id;
        err["project_id"] = project_id;
        err["error"] = e.what();
        return err;
    }
}

}} // namespace avm_worker::avm_stitch
