#include "common/image_converter.hpp"
#include "common/base64.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <stdexcept>
#include <vector>
#include <cstdint>

namespace avm_worker { namespace avm_stitch {

namespace {
constexpr int kRgbaChannels = 4;
}

DecodedImage DecodeBase64JpegToRgba(const std::string& base64_jpeg) {
    // 1. Base64 → raw JPEG bytes
    std::vector<std::uint8_t> jpeg_bytes = avm_worker::base64_decode(base64_jpeg);
    if (jpeg_bytes.empty()) {
        throw std::runtime_error("Empty or invalid base64 data");
    }

    // 2. JPEG → cv::Mat (BGR)
    cv::Mat bgr = cv::imdecode(jpeg_bytes, cv::IMREAD_COLOR);
    if (bgr.empty()) {
        throw std::runtime_error("Failed to decode JPEG image");
    }

    // 3. BGR → RGBA
    cv::Mat rgba;
    cv::cvtColor(bgr, rgba, cv::COLOR_BGR2RGBA);

    // 4. Extract raw buffer
    DecodedImage result;
    result.width = rgba.cols;
    result.height = rgba.rows;
    size_t total_bytes = static_cast<size_t>(rgba.cols) *
                         static_cast<size_t>(rgba.rows) * kRgbaChannels;
    result.rgba_data.assign(rgba.data, rgba.data + total_bytes);

    return result;
}

}} // namespace avm_worker::avm_stitch
