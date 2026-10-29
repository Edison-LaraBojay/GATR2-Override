// path_assembler.cpp

#include "receiver/feed/path_assembler.h"

namespace spectagatr::receiver::feed
{

bool PathAssembler::offer(const protocol::PathPointRecord& r) {
    if (r.count > kMaxPathPointsReceived) {
        ++refused_;
        return false;
    }
    bool changed = false;
    if (!active_ || r.session != session_ || r.path_id != id_ || r.count != count_) {
        // A new path (or a count that contradicts the one shown: start over).
        active_  = true;
        session_ = r.session;
        id_      = r.path_id;
        count_   = r.count;
        known_   = 0;
        points_.assign(count_, std::nullopt);
        changed = true;
    }
    if (count_ == 0) {
        return changed;
    }
    std::optional<PathPoint>& slot = points_[r.index];
    const PathPoint           point{r.x_mm, r.y_mm};
    if (!slot) {
        slot = point;
        ++known_;
        return true;
    }
    if (slot->x_mm != point.x_mm || slot->y_mm != point.y_mm) {
        slot = point; // the same path id should never move a point; the newest line wins
        return true;
    }
    return changed;
}

bool PathAssembler::clear() {
    const bool shown = active_;
    active_          = false;
    session_         = 0;
    id_              = 0;
    count_           = 0;
    known_           = 0;
    points_.clear();
    return shown;
}

} // namespace spectagatr::receiver::feed
