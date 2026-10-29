// pose_line.cpp

#include "spectagatr/protocol/pose_line.h"

#include <cstdio>

#include "fields.h"

namespace spectagatr::protocol
{
namespace
{

constexpr std::size_t kFieldCount = 10; // after the marker, before '*'

bool angleInRange(Centidegrees v) { return v > -kHalfTurnCdeg && v <= kHalfTurnCdeg; }

bool coordinateInRange(Millimeters v) { return v >= -kMaxCoordinateMm && v <= kMaxCoordinateMm; }

// The marker letters the scanner accepts.
bool markerKind(uint8_t byte) { return byte == 'P' || byte == 'R' || byte == 'T'; }

} // namespace

bool operator==(const PoseRecord& a, const PoseRecord& b) {
    return a.session == b.session && a.epoch == b.epoch && a.seq == b.seq && a.t_ms == b.t_ms &&
           a.x_mm == b.x_mm && a.y_mm == b.y_mm && a.yaw_cdeg == b.yaw_cdeg &&
           a.roll_cdeg == b.roll_cdeg && a.pitch_cdeg == b.pitch_cdeg && a.flags == b.flags;
}

bool valid(const PoseRecord& r) {
    return r.session != 0 && coordinateInRange(r.x_mm) && coordinateInRange(r.y_mm) &&
           angleInRange(r.yaw_cdeg) && angleInRange(r.roll_cdeg) && angleInRange(r.pitch_cdeg);
}

uint16_t crc16(std::string_view bytes) {
    uint16_t crc = 0xFFFF;
    for (char c : bytes) {
        crc ^= static_cast<uint16_t>(static_cast<uint8_t>(c) << 8);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

std::size_t encodeLine(const PoseRecord& r, char* out, std::size_t cap) {
    if (!valid(r) || out == nullptr || cap == 0) {
        return 0;
    }
    char      body[kMaxLineLength];
    const int n =
        std::snprintf(body, sizeof(body), "SGP1 %08lX %u %lu %lu %ld %ld %ld %ld %ld %u",
                      static_cast<unsigned long>(r.session), static_cast<unsigned>(r.epoch),
                      static_cast<unsigned long>(r.seq), static_cast<unsigned long>(r.t_ms),
                      static_cast<long>(r.x_mm), static_cast<long>(r.y_mm),
                      static_cast<long>(r.yaw_cdeg), static_cast<long>(r.roll_cdeg),
                      static_cast<long>(r.pitch_cdeg), static_cast<unsigned>(r.flags));
    if (n <= 0 || static_cast<std::size_t>(n) >= sizeof(body)) {
        return 0;
    }
    return fields::closeFrame(body, static_cast<std::size_t>(n), out, cap);
}

DecodeResult decodeLine(std::string_view line) {
    DecodeResult     result;
    std::string_view payload;
    result.error = fields::openFrame(line, 'P', payload);
    if (result.error != DecodeError::kNone) {
        return result;
    }
    result.error = DecodeError::kMalformed;

    std::string_view f[kFieldCount];
    if (!fields::split(payload, f, kFieldCount)) {
        return result;
    }
    PoseRecord r;
    uint64_t   epoch = 0, seq = 0, t_ms = 0, flags = 0;
    if (!fields::parseHex(f[0], 8, r.session) || !fields::parseUnsigned(f[1], 0xFFFF, epoch) ||
        !fields::parseUnsigned(f[2], 0xFFFFFFFFull, seq) ||
        !fields::parseUnsigned(f[3], 0xFFFFFFFFull, t_ms) || !fields::parseSigned(f[4], r.x_mm) ||
        !fields::parseSigned(f[5], r.y_mm) || !fields::parseSigned(f[6], r.yaw_cdeg) ||
        !fields::parseSigned(f[7], r.roll_cdeg) || !fields::parseSigned(f[8], r.pitch_cdeg) ||
        !fields::parseUnsigned(f[9], 0xFF, flags)) {
        return result;
    }
    r.epoch       = static_cast<Epoch>(epoch);
    r.seq         = static_cast<Sequence>(seq);
    r.t_ms        = static_cast<SourceMillis>(t_ms);
    r.flags       = static_cast<uint8_t>(flags);
    result.record = r;
    result.error  = valid(r) ? DecodeError::kNone : DecodeError::kOutOfRange;
    return result;
}

const char* toString(DecodeError error) {
    switch (error) {
    case DecodeError::kNone:
        return "none";
    case DecodeError::kMalformed:
        return "malformed";
    case DecodeError::kBadCrc:
        return "bad crc";
    case DecodeError::kOutOfRange:
        return "out of range";
    }
    return "unknown";
}

LineKind lineKind(std::string_view line) {
    if (line.size() < kMarker.size() || line[0] != 'S' || line[1] != 'G' || line[3] != '1' ||
        line[4] != ' ') {
        return LineKind::kUnknown;
    }
    switch (line[kMarkerKindAt]) {
    case 'P':
        return LineKind::kPose;
    case 'R':
        return LineKind::kRobot;
    case 'T':
        return LineKind::kPath;
    default:
        return LineKind::kUnknown;
    }
}

void LineScanner::reset() {
    length_     = 0;
    matched_    = 0;
    collecting_ = false;
}

bool LineScanner::push(uint8_t byte) {
    // The marker is looked for in every byte, so a cut line followed by a new
    // marker loses only the cut line. Position kMarkerKindAt takes any kind.
    const bool fits = matched_ == kMarkerKindAt
                          ? markerKind(byte)
                          : byte == static_cast<uint8_t>(kMarker[matched_]);
    if (fits) {
        if (matched_ == kMarkerKindAt) {
            kind_ = static_cast<char>(byte);
        }
        ++matched_;
    } else {
        matched_ = byte == static_cast<uint8_t>(kMarker[0]) ? 1 : 0;
    }
    if (matched_ == kMarker.size()) {
        if (collecting_) {
            ++stats_.truncated;
        }
        for (std::size_t i = 0; i < kMarker.size(); ++i) {
            line_[i] = kMarker[i];
        }
        line_[kMarkerKindAt] = kind_;
        length_              = kMarker.size();
        matched_             = 0;
        collecting_          = true;
        return false;
    }
    if (!collecting_) {
        return false;
    }

    if (byte == '\n') {
        collecting_ = false;
        if (length_ > 0 && line_[length_ - 1] == '\r') {
            --length_;
        }
        ++stats_.lines;
        return true;
    }
    if (byte == 0x00) {
        ++stats_.truncated; // COBS packet or read ended inside the line
        reset();
        return false;
    }
    if ((byte < 0x20 && byte != '\r') || byte >= 0x7F) {
        ++stats_.garbage;
        reset();
        return false;
    }
    if (length_ + 1 >= kMaxLineLength) { // keep room for the '\n'
        ++stats_.too_long;
        reset();
        return false;
    }
    line_[length_++] = static_cast<char>(byte);
    return false;
}

} // namespace spectagatr::protocol
