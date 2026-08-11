#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <array>
#include <stdexcept>

namespace avm_worker {

consteval std::array<int, 256> BuildBase64Rev() {
    std::array<int, 256> rev{};
    for (int i = 0; i < 256; ++i) rev[i] = -1;
    constexpr const char* kChars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    for (int i = 0; i < 64; ++i) {
        rev[static_cast<unsigned char>(kChars[i])] = i;
    }
    return rev;
}

inline std::vector<uint8_t> base64_decode(const std::string& input) {
    if (input.empty()) return {};

    static constexpr auto kRev = BuildBase64Rev();

    size_t valid_chars = 0;
    size_t padding = 0;
    for (char ch : input) {
        if (ch == '=') {
            ++padding;
            ++valid_chars;
        } else if (kRev[static_cast<unsigned char>(ch)] != -1) {
            ++valid_chars;
        }
    }

    if (valid_chars == 0) return {};

    size_t output_size = (valid_chars * 3) / 4 - padding;
    if (output_size > 10 * 1024 * 1024) {
        throw std::runtime_error("Base64 decoded size exceeds maximum (10MB)");
    }

    std::vector<uint8_t> result;
    result.reserve(output_size);

    int buffer = 0;
    int bits_collected = 0;

    for (char ch : input) {
        if (ch == '=') break;

        int value = kRev[static_cast<unsigned char>(ch)];
        if (value == -1) continue;

        buffer = (buffer << 6) | value;
        bits_collected += 6;

        if (bits_collected >= 8) {
            bits_collected -= 8;
            result.push_back(static_cast<uint8_t>((buffer >> bits_collected) & 0xFF));
        }
    }

    return result;
}

inline std::string base64_encode(const std::vector<uint8_t>& input) {
    static constexpr const char kChars[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string result;
    result.reserve(((input.size() + 2) / 3) * 4);

    size_t i = 0;
    while (i + 3 <= input.size()) {
        uint32_t b = (static_cast<uint32_t>(input[i]) << 16) |
                     (static_cast<uint32_t>(input[i + 1]) << 8) |
                     static_cast<uint32_t>(input[i + 2]);
        result += kChars[(b >> 18) & 0x3F];
        result += kChars[(b >> 12) & 0x3F];
        result += kChars[(b >> 6) & 0x3F];
        result += kChars[b & 0x3F];
        i += 3;
    }

    size_t remaining = input.size() - i;
    if (remaining == 1) {
        uint32_t b = static_cast<uint32_t>(input[i]);
        result += kChars[(b >> 2) & 0x3F];
        result += kChars[(b << 4) & 0x3F];
        result += "==";
    } else if (remaining == 2) {
        uint32_t b = (static_cast<uint32_t>(input[i]) << 8) |
                      static_cast<uint32_t>(input[i + 1]);
        result += kChars[(b >> 10) & 0x3F];
        result += kChars[(b >> 4) & 0x3F];
        result += kChars[(b << 2) & 0x3F];
        result += "=";
    }
    return result;
}

} // namespace avm_worker
