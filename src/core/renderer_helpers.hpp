#pragma once

#include "common/image_converter.hpp"  // DecodedImage

#include <nlohmann/json.hpp>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace avm_worker { namespace avm_stitch {

class FboManager;  // fwd decl: ReadbackViewAsBase64Png 仅需引用

constexpr int kOutputWidth = 960;
constexpr int kOutputHeight = 640;

// Build the standard stitch_error JSON envelope (same contract as the
// subprocess/OSG backends).
nlohmann::json MakeStitchError(const std::string& task_id,
                               const std::string& project_id,
                               const std::string& message);

// Case-insensitive ASCII string comparison.
bool iequals(const std::string& a, const std::string& b);

// Match the calibration camera whose id or label equals `role`
// (left/right/front/rear, case-insensitive) and return the key under which
// its image lives in the task's `images` object. Returns "" when no match.
std::string FindImageKeyForRole(const nlohmann::json& images,
                                const nlohmann::json& calibration,
                                const std::string& role);

// Decode the 4 camera images in [left, right, front, rear] order (the
// AVM_INPUT_FRAME layout) into `storage` and return non-owning RGBA pointers.
// Throws if a camera image is missing or the decoded sizes disagree.
std::vector<const std::uint8_t*> DecodeRoleBuffers(
        const nlohmann::json& images,
        const nlohmann::json& calibration, int& w, int& h,
        std::array<DecodedImage, 4>& storage);

// Save the 4 decoded fisheye images as PNGs under tmp_base for debugging.
// Writes {tmp_base}/{role}.png for role in [left, right, front, rear].
// Best-effort: a write failure is reported to stderr and does NOT throw
// (debugging aid must not break the stitch pipeline).
void SaveDecodedRoleImages(const std::array<DecodedImage, 4>& imgs,
                           const std::string& tmp_base);

// Run model_creator for the three model kinds (bowl_boat / aerial / distort).
// Throws on a non-zero exit code.
void RunModelCreator(const std::string& tmp_base);

// Copy RESOURCES_DIR into tmp_base/resources and strip .template files.
void CopyResources(const std::string& tmp_base);

// Read back the FBO color attachment, flip vertically (OpenGL bottom-left
// origin → PNG top-left origin), convert RGBA→BGR, encode as PNG, and
// base64-encode it for the result envelope.
std::string ReadbackViewAsBase64Png(FboManager& fbo, int w, int h);

}} // namespace avm_worker::avm_stitch
