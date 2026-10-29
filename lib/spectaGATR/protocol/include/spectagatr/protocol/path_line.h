// path_line.h
// Path point line: one point of the planned path the viewer draws, sent point
// by point. The publisher repeats the whole path at a low rate, so a lost
// line is filled in on the next pass; the receiver joins the points it has
// in index order.
//
//   SGT1 <session> <path_id> <index> <count> <x_mm> <y_mm>*<crc>\n
//
//   session   as in the pose line, the same publisher run
//   path_id   decimal 0..65535, +1 per new path or clear
//   index     decimal, 0..count-1
//   count     decimal 0..65535, points in this path; 0 = no path (then
//             index, x and y are 0)
//   x, y      field position, millimeters, as in the pose line
//
// Frame, CRC and scanning: pose_line.h.

#pragma once
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "spectagatr/protocol/pose_line.h"

namespace spectagatr::protocol
{

using PathId    = uint16_t;
using PathIndex = uint16_t;

struct PathPointRecord {
    SessionId   session = 0;
    PathId      path_id = 0;
    PathIndex   index   = 0;
    PathIndex   count   = 0;
    Millimeters x_mm    = 0;
    Millimeters y_mm    = 0;
};

bool operator==(const PathPointRecord& a, const PathPointRecord& b);

bool valid(const PathPointRecord& record);

// As encodeLine / decodeLine for poses.
std::size_t encodePathLine(const PathPointRecord& record, char* out, std::size_t cap);

struct PathDecodeResult {
    DecodeError     error = DecodeError::kMalformed;
    PathPointRecord record;
};

PathDecodeResult decodePathLine(std::string_view line);

} // namespace spectagatr::protocol
