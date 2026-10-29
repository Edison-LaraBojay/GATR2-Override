// pose_feed.h
// Source bytes to accepted poses, robot footprint and path: LineScanner -> decode by line kind ->
// PoseTracker or PathAssembler, the one path every source (controller, Brain USB, sim, replay)
// goes through. Pure.
//
// The robot and path belong to the publisher run that sent them: a line of another session, of
// any kind, is a new run and drops what the old one sent.

#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

#include "receiver/feed/path_assembler.h"
#include "receiver/feed/pose_tracker.h"
#include "spectagatr/protocol/pose_line.h"
#include "spectagatr/protocol/robot_line.h"

namespace spectagatr::receiver::feed
{

struct FeedCounts {
    uint64_t poses        = 0;
    uint64_t bad_crc      = 0;
    uint64_t malformed    = 0;
    uint64_t out_of_range = 0;
    uint64_t repeated     = 0;
    uint64_t out_of_order = 0;
    uint64_t truncated    = 0;
    uint64_t too_long     = 0;
    uint64_t garbage      = 0;
    uint64_t sessions     = 0;
    uint64_t bytes        = 0;
    uint64_t robot_lines  = 0; // accepted
    uint64_t path_points  = 0; // accepted path lines
};

bool operator==(const FeedCounts& a, const FeedCounts& b);

// What changed besides poses, per push.
struct AuxChange {
    bool robot = false; // robot() changed
    bool path  = false; // path() changed
};

class PoseFeed {
  public:
    // Bytes as they arrived at rx_ms; on_pose(const TrackedPose&) per accepted pose. The robot
    // and path changes are reported through the returned AuxChange.
    template <typename OnPose>
    AuxChange push(const uint8_t* data, std::size_t size, RxMillis rx_ms, OnPose&& on_pose) {
        AuxChange change;
        bytes_ += size;
        scanner_.feed(data, size, [&](std::string_view line) {
            switch (protocol::lineKind(line)) {
            case protocol::LineKind::kPose: {
                const std::optional<TrackedPose> pose =
                    tracker_.offer(protocol::decodeLine(line), rx_ms);
                if (pose) {
                    if (pose->restart) {
                        adoptSession(pose->record.session, change);
                    }
                    on_pose(*pose);
                }
                break;
            }
            case protocol::LineKind::kRobot:
                offerRobot(protocol::decodeRobotLine(line), change);
                break;
            case protocol::LineKind::kPath:
                offerPath(protocol::decodePathLine(line), change);
                break;
            case protocol::LineKind::kUnknown:
                break;
            }
        });
        return change;
    }

    // The source reconnected: a partial line from before is not continued.
    void resetTransport() { scanner_.reset(); }

    // A replayed stream rewound.
    void forgetOrdering() { tracker_.forgetOrdering(); }

    FeedCounts              counts() const;
    std::optional<RxMillis> lastRx() const { return tracker_.lastRx(); }

    // The footprint the current publisher run sent, if any.
    const std::optional<protocol::RobotRecord>& robot() const { return robot_; }
    const PathAssembler&                        path() const { return path_; }

  private:
    void adoptSession(protocol::SessionId session, AuxChange& change);
    void offerRobot(const protocol::RobotDecodeResult& decoded, AuxChange& change);
    void offerPath(const protocol::PathDecodeResult& decoded, AuxChange& change);
    void countError(protocol::DecodeError error);

    protocol::LineScanner                scanner_;
    PoseTracker                          tracker_;
    PathAssembler                        path_;
    std::optional<protocol::RobotRecord> robot_;
    protocol::SessionId                  session_ = 0; // of the newest line of any kind
    uint64_t                             bytes_   = 0;
    uint64_t                             robot_lines_  = 0;
    uint64_t                             path_points_  = 0;
    uint64_t                             bad_crc_      = 0; // of robot and path lines
    uint64_t                             malformed_    = 0;
    uint64_t                             out_of_range_ = 0;
};

} // namespace spectagatr::receiver::feed
