#pragma once

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <string>
#include <string_view>

namespace avm_worker { namespace avm_stitch {

// Canonical camera role tags: the capitalized forms used by the calib XML
// templates' placeholders ({{FX_Front}}) and by index-based fallback.
// Single source for both role matching and the legacy index slot order.
inline constexpr std::string_view kRoleTags[] = {"Front", "Rear", "Left",
                                                 "Right"};

// Case-insensitive match of a raw camera id/label against the canonical role
// tags. Returns the canonical tag view ("Front"...) on hit, empty on miss.
inline std::string_view CanonicalRoleTag(std::string_view raw) {
    for (std::string_view tag : kRoleTags) {
        if (tag.size() == raw.size() &&
            std::equal(tag.begin(), tag.end(), raw.begin(),
                       [](char a, char b) {
                           return std::tolower(static_cast<unsigned char>(a)) ==
                                  std::tolower(static_cast<unsigned char>(b));
                       })) {
            return tag;
        }
    }
    return {};
}

// Resolve the label-style name used for subprocess OUTPUT filenames and XML
// tags (e.g. Front.png, {{FX_Front}}). Resolution order:
// 1. cam["id"] role match (real-car main path): on the real-car link
//    calibration.cameras[].id IS the role (front/rear/left/right,
//    system-guaranteed), while the label is a user-configured display name
//    (often Chinese, e.g. "前置相机") with no role semantics — so the id must
//    resolve the tag first, regardless of the label.
// 2. cam["label"] role match (sim fallback): sim ids are cam_N (no role
//    semantics), so only the label can name the role there. Placeholders are
//    replaced case-sensitively, so a role-naming label in any case
//    ("front"/"FRONT") must normalize to the canonical tag.
// 3. Non-role label: returned unchanged. Such user labels cannot match the
//    templates' role placeholders; EnsureNoUnreplacedPlaceholders in
//    CalibFilesGenerator turns that into an explicit, visible error
//    (production incident: silent black frames before the fail-fast).
// 4./5. Legacy Front/Rear/Left/Right by index (i<4), else "CAM{i}".
inline std::string ResolveCameraLabel(const nlohmann::json& cam, size_t index) {
    if (cam.contains("id") && cam["id"].is_string()) {
        const std::string_view role =
            CanonicalRoleTag(cam["id"].get<std::string>());
        if (!role.empty()) return std::string(role);
    }
    if (cam.contains("label") && cam["label"].is_string() &&
        !cam["label"].get<std::string>().empty()) {
        const std::string label = cam["label"].get<std::string>();
        const std::string_view role = CanonicalRoleTag(label);
        if (!role.empty()) return std::string(role);
        return label;
    }
    if (index < 4) return std::string(kRoleTags[index]);
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
