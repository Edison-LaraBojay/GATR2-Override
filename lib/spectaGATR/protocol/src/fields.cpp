// fields.cpp

#include "fields.h"

#include <cstdio>

namespace spectagatr::protocol::fields
{
namespace
{

constexpr std::size_t kCrcDigits = 4;

int hexValue(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

} // namespace

bool parseHex(std::string_view text, std::size_t digits, uint32_t& out) {
    if (text.size() != digits) {
        return false;
    }
    uint32_t value = 0;
    for (char c : text) {
        const int v = hexValue(c);
        if (v < 0) {
            return false;
        }
        value = (value << 4) | static_cast<uint32_t>(v);
    }
    out = value;
    return true;
}

bool parseUnsigned(std::string_view text, uint64_t max, uint64_t& out) {
    if (text.empty() || text.size() > 10) {
        return false;
    }
    uint64_t value = 0;
    for (char c : text) {
        if (c < '0' || c > '9') {
            return false;
        }
        value = value * 10 + static_cast<uint64_t>(c - '0');
    }
    if (value > max) {
        return false;
    }
    out = value;
    return true;
}

bool parseSigned(std::string_view text, int32_t& out) {
    const bool negative = !text.empty() && text.front() == '-';
    if (negative) {
        text.remove_prefix(1);
    }
    uint64_t magnitude = 0;
    if (!parseUnsigned(text, negative ? 2147483648ull : 2147483647ull, magnitude)) {
        return false;
    }
    out = negative ? static_cast<int32_t>(-static_cast<int64_t>(magnitude))
                   : static_cast<int32_t>(magnitude);
    return true;
}

bool split(std::string_view text, std::string_view* out, std::size_t count) {
    std::size_t n = 0;
    while (true) {
        const std::size_t      space = text.find(' ');
        const std::string_view field = text.substr(0, space);
        if (field.empty() || n == count) {
            return false;
        }
        out[n++] = field;
        if (space == std::string_view::npos) {
            break;
        }
        text.remove_prefix(space + 1);
    }
    return n == count;
}

DecodeError openFrame(std::string_view line, char kind, std::string_view& payload) {
    if (!line.empty() && line.back() == '\r') {
        line.remove_suffix(1);
    }
    if (line.size() < kMarker.size() + 1 + kCrcDigits || line[0] != 'S' || line[1] != 'G' ||
        line[2] != kind || line[3] != '1' || line[4] != ' ') {
        return DecodeError::kMalformed;
    }
    const std::size_t star = line.size() - kCrcDigits - 1;
    if (line[star] != '*') {
        return DecodeError::kMalformed;
    }
    uint32_t sent_crc = 0;
    if (!parseHex(line.substr(star + 1), kCrcDigits, sent_crc)) {
        return DecodeError::kMalformed;
    }
    const std::string_view body = line.substr(0, star);
    if (crc16(body) != sent_crc) {
        return DecodeError::kBadCrc;
    }
    payload = body.substr(kMarker.size());
    return DecodeError::kNone;
}

std::size_t closeFrame(const char* body, std::size_t length, char* out, std::size_t cap) {
    if (length == 0 || out == nullptr) {
        return 0;
    }
    const uint16_t    crc   = crc16(std::string_view(body, length));
    const std::size_t total = length + 1 + kCrcDigits + 1;
    if (total > kMaxLineLength || total >= cap) {
        return 0;
    }
    const int written =
        std::snprintf(out, cap, "%.*s*%04X\n", static_cast<int>(length), body,
                      static_cast<unsigned>(crc));
    return written == static_cast<int>(total) ? total : 0;
}

} // namespace spectagatr::protocol::fields
