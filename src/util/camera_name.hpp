#pragma once

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <string>
#include <string_view>

namespace avm_worker { namespace avm_stitch {

// Resolve the label-style name used for subprocess OUTPUT filenames and XML
// tags (e.g. Front.png, {{FX_Front}}). Resolution order:
// cam["label"] -> legacy Front/Rear/Left/Right by index (i<4) -> "CAM{i}".
// The id is deliberately NOT used: the API keys images by lowercase id
// ("front"), but files/tags must use the capitalized label ("Front") to
// match the calib XML templates and the user's file naming requirement.
inline std::string ResolveCameraLabel(const nlohmann::json& cam, size_t index) {
    static const char* legacy_tags[] = {"Front", "Rear", "Left", "Right"};
    if (cam.contains("label") && cam["label"].is_string() &&
        !cam["label"].get<std::string>().empty()) {
        const std::string label = cam["label"].get<std::string>();
        // Incoming labels may use any case ("front"/"FRONT"), but the calib
        // XML templates' placeholders are capitalized ({{FX_Front}}) and are
        // replaced case-sensitively. A label naming a known role must be
        // normalized to the canonical tag, otherwise nothing gets replaced
        // and model_creator parses the leftover {{...}} literals as DBL_MAX
        // (production incident: silent black frames). Other labels pass
        // through unchanged (the placeholder fail-fast in
        // CalibFilesGenerator makes such mismatches a visible error).
        for (const char* tag : legacy_tags) {
            const std::string_view t(tag);
            if (t.size() == label.size() &&
                std::equal(t.begin(), t.end(), label.begin(),
                           [](char a, char b) {
                               return std::tolower(static_cast<unsigned char>(
                                          a)) ==
                                      std::tolower(static_cast<unsigned char>(
                                          b));
                           })) {
                return std::string(t);
            }
        }
        return label;
    }
    if (index < 4) return legacy_tags[index];
    return "CAM" + std::to_string(index);
}

// Resolve the image key for a calibration camera entry (OSG backend).
// Resolution order: cam["id"] -> cam["label"] (raw) -> ResolveCameraLabel's
// index fallback. The label is returned WITHOUT case canonicalization:
// image keys mirror how the API addresses cameras (id-style), and image-key
// matching (FindCameraImageKey, renderer camera roles) is case-insensitive
// anyway — only the canonical XML-tag resolution (ResolveCameraLabel)
// normalizes the case.
inline std::string ResolveCameraName(const nlohmann::json& cam, size_t index) {
    if (cam.contains("id") && cam["id"].is_string() &&
        !cam["id"].get<std::string>().empty()) {
        return cam["id"].get<std::string>();
    }
    if (cam.contains("label") && cam["label"].is_string() &&
        !cam["label"].get<std::string>().empty()) {
        return cam["label"].get<std::string>();
    }
    return ResolveCameraLabel(cam, index);
}

}} // namespace avm_worker::avm_stitch
