#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace avm_worker { namespace avm_stitch {

/// Result of base64 JPEG → RGBA8888 conversion.
struct DecodedImage {
    std::vector<std::uint8_t> rgba_data; // RGBA interleaved, row-major
    int width = 0;
    int height = 0;
};

/// Decode a base64-encoded JPEG to a raw RGBA8888 pixel buffer.
/// @param base64_jpeg  base64-encoded JPEG image data
/// @return DecodedImage with RGBA data, width, and height
/// @throws std::runtime_error on decode failure
DecodedImage DecodeBase64JpegToRgba(const std::string& base64_jpeg);

}} // namespace avm_worker::avm_stitch
