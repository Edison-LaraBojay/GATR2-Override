// feed_json.h
// The receiver-to-browser messages of PROTOCOL.md as JSON text: hello, pose, robot, path, status,
// and the /api/fields listing. Numbers are SI (meters, radians); a non-finite number is written as
// null. Strings are escaped to plain ASCII, so a WebSocket text frame is always valid UTF-8. Pure.

#pragma once
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "receiver/feed/feed_status.h"
#include "receiver/feed/path_assembler.h"
#include "receiver/feed/pose_tracker.h"
#include "spectagatr/protocol/robot_line.h"

namespace spectagatr::receiver::feed
{

using Meters     = double;
using FieldNames = std::vector<std::string>;

constexpr int         kFeedProtocol = 1;
constexpr const char* kReceiverName = "spectagatr 1.0";

// Visual robot box around the pose point: the receiver's default, shown until the Brain sends its
// own footprint.
struct RobotBox {
    Meters front  = 0.1685;
    Meters back   = 0.1685;
    Meters left   = 0.1465;
    Meters right  = 0.1465;
    Meters height = 0.25;
};

struct HelloInfo {
    int64_t     stale_ms = 750;
    std::string field    = "override";
    FieldNames  fields;
    RobotBox    robot;
};

std::string helloJson(const HelloInfo& hello);
std::string poseJson(const TrackedPose& pose, std::string_view source);
// The footprint the Brain sent; "session" null and the default box when it sent none (a new run).
std::string robotJson(const protocol::RobotRecord* robot, const RobotBox& fallback);
// The path as assembled so far; count 0 and no points when there is none.
std::string pathJson(const PathAssembler& path);
std::string statusJson(const FeedStatus& status);
std::string fieldsJson(const FieldNames& fields, const std::string& default_field);

// "text" with quotes; ", \ and bytes outside 0x20..0x7E escaped.
void appendJsonString(std::string& out, std::string_view text);

// Fixed decimals, trailing zeros dropped, -0 as 0; null when not finite.
void appendJsonNumber(std::string& out, double value, int decimals);

} // namespace spectagatr::receiver::feed
