// publisher.h
// Portable publisher core: the newest pose in a lock-free slot, pulled as SGP1
// lines by one transport task; footprint and path as low priority SGR1 and
// SGT1 lines. The PROS transport is viewer.h.
//
// updatePose, resetTrail, setRobot, setPath, clearPath: any task, no blocking,
// no allocation. setPath from one task at a time.
// begin, nextLine, nextAuxLine, recordWrite: the transport task only.

#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "spectagatr/pose.h"
#include "spectagatr/protocol/path_line.h"
#include "spectagatr/protocol/pose_line.h"
#include "spectagatr/protocol/robot_line.h"

namespace spectagatr
{

using protocol::Epoch;
using protocol::PathId;
using protocol::Sequence;
using protocol::SessionId;

// Publisher clock in milliseconds; pros::millis on the Brain.
using Clock = Milliseconds (*)();

constexpr Hertz        kDefaultRateHz    = 10;
constexpr Hertz        kMinRateHz        = 1;
constexpr Hertz        kMaxRateHz        = 50;
constexpr Meters       kMaxCoordinate    = protocol::kMaxCoordinateMm / 1000.0; // 2000 m
constexpr Meters       kMaxRobotSize     = protocol::kMaxRobotMm / 1000.0;      // 5 m
constexpr std::size_t  kLineCapacity     = protocol::kMaxLineLength + 1;        // line plus NUL
constexpr std::size_t  kMaxPathPoints    = 256;
constexpr Milliseconds kRobotRepeatMs    = 2000; // the footprint line goes again this often
constexpr Milliseconds kPathRepeatMs     = 2000; // a whole path goes again about this often
constexpr Milliseconds kPathMinSpacingMs = 50;   // between repeated path lines

struct PublisherStats {
    uint32_t updates        = 0; // accepted updatePose calls
    uint32_t rejected       = 0; // non-finite or out of range; the previous value stays
    uint32_t lines          = 0; // pose lines handed to the transport
    uint32_t overwritten    = 0; // accepted poses replaced before a line carried them
    uint32_t write_errors   = 0; // failed or short transport writes
    uint32_t robot_lines    = 0;
    uint32_t path_lines     = 0;
    uint32_t paths          = 0; // accepted setPath and clearPath calls
    uint32_t path_truncated = 0; // points beyond kMaxPathPoints, dropped
};

// Nonzero session id hashed (splitmix64) from values that differ between runs.
SessionId makeSessionId(const uint64_t* inputs, std::size_t count);

class Publisher {
  public:
    explicit Publisher(Clock clock, Hertz rate = kDefaultRateHz);

    Publisher(const Publisher&)            = delete;
    Publisher& operator=(const Publisher&) = delete;

    // Heading only: stored as Attitude{0, 0, heading}, full attitude flag off.
    void updatePose(Meters x, Meters y, Radians heading);
    // Full attitude, flag on.
    void updatePose(Meters x, Meters y, const Attitude& attitude);

    // Explicit pose reset or re-placement: poses supplied after this call
    // carry the next epoch and the viewer starts a new trail.
    void resetTrail();

    // The box drawn around the pose point. Sent at once, then every
    // kRobotRepeatMs. A box with no length, width or height, or over
    // kMaxRobotSize, is rejected.
    void setRobot(const RobotBox& box);

    // The planned path, points in order; count 0 clears. The first
    // kMaxPathPoints are kept. Any non-finite or out-of-range point rejects
    // the whole call. The transport sends every point once as fast as the
    // output allows, then repeats the path about every kPathRepeatMs so a
    // lost line is filled in.
    void setPath(const Point* points, std::size_t count);
    // The same through an accessor point_at(i) -> Point, for any container.
    template <typename PointAt> void setPath(std::size_t count, PointAt&& point_at) {
        const std::size_t stored = count > kMaxPathPoints ? kMaxPathPoints : count;
        for (std::size_t i = 0; i < stored; ++i) {
            if (!validPoint(point_at(i))) {
                rejected_.fetch_add(1, std::memory_order_relaxed);
                return;
            }
        }
        if (stored < count) {
            path_truncated_.fetch_add(static_cast<uint32_t>(count - stored),
                                      std::memory_order_relaxed);
        }
        beginPath();
        for (std::size_t i = 0; i < stored; ++i) {
            writePathPoint(i, point_at(i));
        }
        endPath(stored);
    }
    void clearPath();

    // Clamped to [kMinRateHz, kMaxRateHz]; non-finite gives the default.
    void         setRate(Hertz rate);
    Milliseconds period() const; // minimum spacing of pose lines, round(1000 / rate)
    Hertz        rate() const;   // 1000 / period

    PublisherStats stats() const;
    SessionId      session() const; // 0 before begin

    // Transport task. Sets the session (nonzero) and the next line's seq. The
    // robot and path go out again under the new session.
    void begin(SessionId session, Sequence first_seq = 0);

    // Writes the next pose line, '\n' included, and returns its length; 0
    // when there is no session yet, no new pose since the last line, the
    // period has not elapsed, a writer is busy, or cap < kLineCapacity.
    std::size_t nextLine(Milliseconds now, char* out, std::size_t cap);

    // Writes the next robot or path line that is due, '\n' included, and
    // returns its length; 0 when none is due. Call it after nextLine, only
    // when the output is empty: pose lines come first.
    std::size_t nextAuxLine(Milliseconds now, char* out, std::size_t cap);

    // Transport task, after writing any line: false for a failed or short write.
    void recordWrite(bool ok);

  private:
    static constexpr std::size_t kSlotWords  = 8; // 7 data words and a checksum
    static constexpr std::size_t kRobotWords = 5;

    static bool validPoint(const Point& point);

    void store(Meters x, Meters y, const Attitude& attitude, bool full_attitude);
    bool load(uint32_t& version, uint32_t (&words)[kSlotWords]) const;

    bool loadRobot(uint32_t& version, protocol::RobotRecord& record) const;

    void beginPath();
    void writePathPoint(std::size_t index, const Point& point);
    void endPath(std::size_t count);
    // One point of the stored path (count 0: the empty record). False while
    // a writer is busy or the path changed under the read.
    bool loadPathPoint(std::size_t index, uint32_t& version, protocol::PathPointRecord& record) const;

    std::size_t robotLine(Milliseconds now, char* out, std::size_t cap);
    std::size_t pathLine(Milliseconds now, char* out, std::size_t cap);

    Clock clock_;

    // Pose slot: seqlock over relaxed atomic words; odd while a writer is inside.
    std::atomic<uint32_t> slot_seq_{0};
    std::atomic<uint32_t> slot_[kSlotWords]{};

    // Robot slot, the same way. Words: front, back, left, right, height in mm.
    std::atomic<uint32_t> robot_seq_{0};
    std::atomic<uint32_t> robot_[kRobotWords]{};

    // Path slot, the same way. Meta: count in bits 0..15, path id in bits 16..31.
    std::atomic<uint32_t> path_seq_{0};
    std::atomic<uint32_t> path_meta_{0};
    std::atomic<uint32_t> path_x_[kMaxPathPoints]{};
    std::atomic<uint32_t> path_y_[kMaxPathPoints]{};
    std::atomic<uint32_t> path_id_{0};

    std::atomic<uint32_t> epoch_{0};
    std::atomic<uint32_t> period_ms_{0};
    std::atomic<uint32_t> session_{0};

    std::atomic<uint32_t> updates_{0};
    std::atomic<uint32_t> rejected_{0};
    std::atomic<uint32_t> lines_{0};
    std::atomic<uint32_t> overwritten_{0};
    std::atomic<uint32_t> write_errors_{0};
    std::atomic<uint32_t> robot_lines_{0};
    std::atomic<uint32_t> path_lines_{0};
    std::atomic<uint32_t> paths_{0};
    std::atomic<uint32_t> path_truncated_{0};

    // Transport task only.
    Sequence     next_seq_     = 0;
    uint32_t     sent_version_ = 0; // slot version of the last pose line, 0 = none
    bool         scheduled_    = false;
    Milliseconds due_          = 0;

    uint32_t     robot_sent_version_ = 0;
    bool         robot_scheduled_    = false;
    Milliseconds robot_due_          = 0;

    uint32_t     path_sent_version_ = 0;
    std::size_t  path_cursor_       = 0;
    bool         path_first_pass_   = true;
    bool         path_scheduled_    = false;
    Milliseconds path_due_          = 0;
};

} // namespace spectagatr
