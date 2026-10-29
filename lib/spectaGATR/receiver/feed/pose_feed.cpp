// pose_feed.cpp

#include "receiver/feed/pose_feed.h"

namespace spectagatr::receiver::feed
{

bool operator==(const FeedCounts& a, const FeedCounts& b) {
    return a.poses == b.poses && a.bad_crc == b.bad_crc && a.malformed == b.malformed &&
           a.out_of_range == b.out_of_range && a.repeated == b.repeated &&
           a.out_of_order == b.out_of_order && a.truncated == b.truncated &&
           a.too_long == b.too_long && a.garbage == b.garbage && a.sessions == b.sessions &&
           a.bytes == b.bytes && a.robot_lines == b.robot_lines && a.path_points == b.path_points;
}

FeedCounts PoseFeed::counts() const {
    const TrackerCounts&                tracked = tracker_.counts();
    const protocol::LineScanner::Stats& scanned = scanner_.stats();
    FeedCounts                          counts;
    counts.poses        = tracked.poses;
    counts.bad_crc      = tracked.bad_crc + bad_crc_;
    counts.malformed    = tracked.malformed + malformed_;
    counts.out_of_range = tracked.out_of_range + out_of_range_;
    counts.repeated     = tracked.repeated;
    counts.out_of_order = tracked.out_of_order;
    counts.truncated    = scanned.truncated;
    counts.too_long     = scanned.too_long;
    counts.garbage      = scanned.garbage;
    counts.sessions     = tracked.sessions;
    counts.bytes        = bytes_;
    counts.robot_lines  = robot_lines_;
    counts.path_points  = path_points_;
    return counts;
}

void PoseFeed::adoptSession(protocol::SessionId session, AuxChange& change) {
    if (session == session_) {
        return;
    }
    session_ = session;
    if (robot_ && robot_->session != session) {
        robot_.reset();
        change.robot = true;
    }
    if (path_.hasPath() && path_.session() != session) {
        path_.clear();
        change.path = true;
    }
}

void PoseFeed::offerRobot(const protocol::RobotDecodeResult& decoded, AuxChange& change) {
    if (decoded.error != protocol::DecodeError::kNone) {
        countError(decoded.error);
        return;
    }
    ++robot_lines_;
    adoptSession(decoded.record.session, change);
    if (!robot_ || !(*robot_ == decoded.record)) {
        robot_       = decoded.record;
        change.robot = true;
    }
}

void PoseFeed::offerPath(const protocol::PathDecodeResult& decoded, AuxChange& change) {
    if (decoded.error != protocol::DecodeError::kNone) {
        countError(decoded.error);
        return;
    }
    ++path_points_;
    adoptSession(decoded.record.session, change);
    if (path_.offer(decoded.record)) {
        change.path = true;
    }
}

void PoseFeed::countError(protocol::DecodeError error) {
    switch (error) {
    case protocol::DecodeError::kBadCrc:
        ++bad_crc_;
        break;
    case protocol::DecodeError::kOutOfRange:
        ++out_of_range_;
        break;
    case protocol::DecodeError::kMalformed:
        ++malformed_;
        break;
    case protocol::DecodeError::kNone:
        break;
    }
}

} // namespace spectagatr::receiver::feed
