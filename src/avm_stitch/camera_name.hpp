#pragma once

#include <nlohmann/json.hpp>
#include <string>

namespace avm_worker { namespace avm_stitch {

// Resolve the label-style name used for subprocess OUTPUT filenames and XML
// tags (e.g. Front.png, {{FX_Front}}). Resolution order:
// cam["label"] -> legacy Front/Rear/Left/Right by index (i<4) -> "CAM{i}".
// The id is deliberately NOT used: the API keys images by lowercase id
// ("front"), but files/tags must use the capitalized label ("Front") to
// match the calib XML templates and the user's file naming requirement.
inline std::string ResolveCameraLabel(const nlohmann::json& cam, size_t index) {
    if (cam.contains("label") && cam["label"].is_string() &&
        !cam["label"].get<std::string>().empty()) {
        return cam["label"].get<std::string>();
    }
    static const char* legacy_tags[] = {"Front", "Rear", "Left", "Right"};
    if (index < 4) return legacy_tags[index];
    return "CAM" + std::to_string(index);
}

// Resolve the image key for a calibration camera entry (OSG backend).
// Resolution order: cam["id"] -> ResolveCameraLabel. The OSG backend's
// DecodeCameraImages matches `images` keys, which are id-style, so id is
// preferred here; when absent, fall back to the label-style resolution.
inline std::string ResolveCameraName(const nlohmann::json& cam, size_t index) {
    if (cam.contains("id") && cam["id"].is_string() &&
        !cam["id"].get<std::string>().empty()) {
        return cam["id"].get<std::string>();
    }
    return ResolveCameraLabel(cam, index);
}

}} // namespace avm_worker::avm_stitch
