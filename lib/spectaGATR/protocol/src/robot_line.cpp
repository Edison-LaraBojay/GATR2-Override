// robot_line.cpp

#include "spectagatr/protocol/robot_line.h"

#include <cstdio>

#include "fields.h"

namespace spectagatr::protocol
{
namespace
{

constexpr std::size_t kFieldCount = 6;

bool sizeInRange(Millimeters v) { return v >= 0 && v <= kMaxRobotMm; }

} // namespace

bool operator==(const RobotRecord& a, const RobotRecord& b) {
    return a.session == b.session && a.front_mm == b.front_mm && a.back_mm == b.back_mm &&
           a.left_mm == b.left_mm && a.right_mm == b.right_mm && a.height_mm == b.height_mm;
}

bool valid(const RobotRecord& r) {
    return r.session != 0 && sizeInRange(r.front_mm) && sizeInRange(r.back_mm) &&
           sizeInRange(r.left_mm) && sizeInRange(r.right_mm) && sizeInRange(r.height_mm) &&
           r.front_mm + r.back_mm > 0 && r.left_mm + r.right_mm > 0 && r.height_mm > 0;
}

std::size_t encodeRobotLine(const RobotRecord& r, char* out, std::size_t cap) {
    if (!valid(r) || out == nullptr || cap == 0) {
        return 0;
    }
    char      body[kMaxLineLength];
    const int n = std::snprintf(body, sizeof(body), "SGR1 %08lX %ld %ld %ld %ld %ld",
                                static_cast<unsigned long>(r.session),
                                static_cast<long>(r.front_mm), static_cast<long>(r.back_mm),
                                static_cast<long>(r.left_mm), static_cast<long>(r.right_mm),
                                static_cast<long>(r.height_mm));
    if (n <= 0 || static_cast<std::size_t>(n) >= sizeof(body)) {
        return 0;
    }
    return fields::closeFrame(body, static_cast<std::size_t>(n), out, cap);
}

RobotDecodeResult decodeRobotLine(std::string_view line) {
    RobotDecodeResult result;
    std::string_view  payload;
    result.error = fields::openFrame(line, 'R', payload);
    if (result.error != DecodeError::kNone) {
        return result;
    }
    result.error = DecodeError::kMalformed;

    std::string_view f[kFieldCount];
    if (!fields::split(payload, f, kFieldCount)) {
        return result;
    }
    RobotRecord r;
    if (!fields::parseHex(f[0], 8, r.session) || !fields::parseSigned(f[1], r.front_mm) ||
        !fields::parseSigned(f[2], r.back_mm) || !fields::parseSigned(f[3], r.left_mm) ||
        !fields::parseSigned(f[4], r.right_mm) || !fields::parseSigned(f[5], r.height_mm)) {
        return result;
    }
    result.record = r;
    result.error  = valid(r) ? DecodeError::kNone : DecodeError::kOutOfRange;
    return result;
}

} // namespace spectagatr::protocol
