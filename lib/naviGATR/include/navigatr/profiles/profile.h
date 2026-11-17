// profile.h
// Stage 2, profile: Path -> Trajectory, speed and time per point. Portable.

#pragma once
#include <vector>

#include "navigatr/config.h"
#include "navigatr/path.h"

namespace navigatr
{

struct TrajectoryPoint {
    double t            = 0; // s from the start of the move
    double s            = 0; // in along the path
    double x            = 0;
    double y            = 0;
    double v            = 0; // in/s along the path
    double tangent      = 0; // deg
    double heading      = 0; // deg, continuous
    double heading_rate = 0; // deg/s
};

struct Trajectory {
    std::vector<TrajectoryPoint> points;

    double duration() const { return points.empty() ? 0.0 : points.back().t; }
    // Past the end: the end, at rest.
    TrajectoryPoint at(double t) const;
};

class Profile {
  public:
    virtual ~Profile() = default;
    virtual const char* name() const = 0;
    virtual bool        begin(const Config&) { return true; }
    virtual Trajectory  build(const Path& path) const = 0;
};

} // namespace navigatr
