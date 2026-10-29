// robot_line.h
// Robot footprint line: the box the viewer draws around the pose point,
// sent by the publisher so the laptop needs no robot settings.
//
//   SGR1 <session> <front_mm> <back_mm> <left_mm> <right_mm> <height_mm>*<crc>\n
//
//   session       as in the pose line, the same publisher run
//   front, back   distance from the pose point along robot +x and -x
//   left, right   along robot +y and -y
//   height        above the floor
//   all           decimal millimeters, 0..kMaxRobotMm; the box must have a
//                 positive length, width and height
//
// Frame, CRC and scanning: pose_line.h.

#pragma once
#include <cstddef>
#include <string_view>

#include "spectagatr/protocol/pose_line.h"

namespace spectagatr::protocol
{

constexpr Millimeters kMaxRobotMm = 5000;

struct RobotRecord {
    SessionId   session   = 0;
    Millimeters front_mm  = 0;
    Millimeters back_mm   = 0;
    Millimeters left_mm   = 0;
    Millimeters right_mm  = 0;
    Millimeters height_mm = 0;
};

bool operator==(const RobotRecord& a, const RobotRecord& b);

bool valid(const RobotRecord& record);

// As encodeLine / decodeLine for poses.
std::size_t encodeRobotLine(const RobotRecord& record, char* out, std::size_t cap);

struct RobotDecodeResult {
    DecodeError error = DecodeError::kMalformed;
    RobotRecord record;
};

RobotDecodeResult decodeRobotLine(std::string_view line);

} // namespace spectagatr::protocol
