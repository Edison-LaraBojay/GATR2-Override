// path_line.cpp

#include "spectagatr/protocol/path_line.h"

#include <cstdio>

#include "fields.h"

namespace spectagatr::protocol
{
namespace
{

constexpr std::size_t kFieldCount = 6;

bool coordinateInRange(Millimeters v) { return v >= -kMaxCoordinateMm && v <= kMaxCoordinateMm; }

} // namespace

bool operator==(const PathPointRecord& a, const PathPointRecord& b) {
    return a.session == b.session && a.path_id == b.path_id && a.index == b.index &&
           a.count == b.count && a.x_mm == b.x_mm && a.y_mm == b.y_mm;
}

bool valid(const PathPointRecord& r) {
    if (r.session == 0 || !coordinateInRange(r.x_mm) || !coordinateInRange(r.y_mm)) {
        return false;
    }
    if (r.count == 0) {
        return r.index == 0 && r.x_mm == 0 && r.y_mm == 0;
    }
    return r.index < r.count;
}

std::size_t encodePathLine(const PathPointRecord& r, char* out, std::size_t cap) {
    if (!valid(r) || out == nullptr || cap == 0) {
        return 0;
    }
    char      body[kMaxLineLength];
    const int n = std::snprintf(body, sizeof(body), "SGT1 %08lX %u %u %u %ld %ld",
                                static_cast<unsigned long>(r.session),
                                static_cast<unsigned>(r.path_id), static_cast<unsigned>(r.index),
                                static_cast<unsigned>(r.count), static_cast<long>(r.x_mm),
                                static_cast<long>(r.y_mm));
    if (n <= 0 || static_cast<std::size_t>(n) >= sizeof(body)) {
        return 0;
    }
    return fields::closeFrame(body, static_cast<std::size_t>(n), out, cap);
}

PathDecodeResult decodePathLine(std::string_view line) {
    PathDecodeResult result;
    std::string_view payload;
    result.error = fields::openFrame(line, 'T', payload);
    if (result.error != DecodeError::kNone) {
        return result;
    }
    result.error = DecodeError::kMalformed;

    std::string_view f[kFieldCount];
    if (!fields::split(payload, f, kFieldCount)) {
        return result;
    }
    PathPointRecord r;
    uint64_t        path_id = 0, index = 0, count = 0;
    if (!fields::parseHex(f[0], 8, r.session) || !fields::parseUnsigned(f[1], 0xFFFF, path_id) ||
        !fields::parseUnsigned(f[2], 0xFFFF, index) ||
        !fields::parseUnsigned(f[3], 0xFFFF, count) || !fields::parseSigned(f[4], r.x_mm) ||
        !fields::parseSigned(f[5], r.y_mm)) {
        return result;
    }
    r.path_id     = static_cast<PathId>(path_id);
    r.index       = static_cast<PathIndex>(index);
    r.count       = static_cast<PathIndex>(count);
    result.record = r;
    result.error  = valid(r) ? DecodeError::kNone : DecodeError::kOutOfRange;
    return result;
}

} // namespace spectagatr::protocol
