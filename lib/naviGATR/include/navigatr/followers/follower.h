// follower.h
// Stage 3, follow: pose + target -> command, every tick. Portable.

#pragma once
#include "navigatr/config.h"
#include "navigatr/types.h"

namespace navigatr
{

// Where the robot should be now, and how fast it should be moving.
struct Target {
    double x         = 0;
    double y         = 0;
    double heading   = 0; // deg
    double vx        = 0; // in/s, field frame
    double vy        = 0; // in/s
    double turn_rate = 0; // deg/s
};

class Follower {
  public:
    virtual ~Follower() = default;
    virtual const char*   name() const = 0;
    virtual bool          begin(const Config&) { return true; }
    virtual void          reset() {} // a new step starts
    virtual ChassisSpeeds update(const Pose& pose, const Target& target, double dt) = 0;
};

} // namespace navigatr
