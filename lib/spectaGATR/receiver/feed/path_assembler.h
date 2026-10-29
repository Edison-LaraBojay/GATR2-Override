// path_assembler.h
// Path point lines (PROTOCOL.md, SGT1) back into one path. Points arrive one
// per line, in any order, repeated; the newest (session, path id) wins and
// every point of it is kept by index until the publisher sends another path
// or clears it. Pure: no clock, no I/O.

#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "spectagatr/protocol/path_line.h"

namespace spectagatr::receiver::feed
{

// Longer paths are refused: a wire count is 16 bits, this is the memory cap.
constexpr std::size_t kMaxPathPointsReceived = 4096;

struct PathPoint {
    protocol::Millimeters x_mm = 0;
    protocol::Millimeters y_mm = 0;
};

class PathAssembler {
  public:
    // One decoded point. True when what the browser should show changed: a
    // new path, a cleared path, or a point not seen before.
    bool offer(const protocol::PathPointRecord& record);

    // A new publisher run: nothing known. True when a path was shown.
    bool clear();

    bool                hasPath() const { return active_; } // a path id is known, count may be 0
    protocol::SessionId session() const { return session_; }
    protocol::PathId    id() const { return id_; }
    protocol::PathIndex count() const { return count_; }
    std::size_t         known() const { return known_; }
    bool                complete() const { return active_ && known_ == count_; }

    // count() entries, none for a point not received yet.
    const std::vector<std::optional<PathPoint>>& points() const { return points_; }

    uint64_t refused() const { return refused_; } // lines of a path over the cap

  private:
    bool                                  active_  = false;
    protocol::SessionId                   session_ = 0;
    protocol::PathId                      id_      = 0;
    protocol::PathIndex                   count_   = 0;
    std::size_t                           known_   = 0;
    uint64_t                              refused_ = 0;
    std::vector<std::optional<PathPoint>> points_;
};

} // namespace spectagatr::receiver::feed
