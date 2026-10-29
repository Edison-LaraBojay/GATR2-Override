// holonomic_follower.cpp

#include "navigatr/followers/holonomic_follower.h"

#include <algorithm>
#include <cmath>

namespace navigatr
{

bool HolonomicFollower::begin(const Config& config) {
    const Gains& g = config.gains;
    x_             = Pid(g.kp, g.ki, g.kd);
    y_             = Pid(g.kp, g.ki, g.kd);
    heading_       = Pid(g.heading_kp, g.heading_ki, g.heading_kd);
    max_turn_rate_ = config.limits.max_turn_rate;
    return max_turn_rate_ > 0;
}

void HolonomicFollower::reset() {
    x_.reset();
    y_.reset();
    heading_.reset();
}

ChassisSpeeds HolonomicFollower::update(const Pose& pose, const Target& target, double dt) {
    // Field frame.
    const double vx   = target.vx + x_.update(target.x - pose.x, dt);
    const double vy   = target.vy + y_.update(target.y - pose.y, dt);
    const double turn = target.turn_rate + heading_.update(wrap180(target.heading - pose.heading), dt);

    // Robot frame.
    const double  h = radians(pose.heading);
    ChassisSpeeds speeds;
    speeds.forward = vx * std::cos(h) + vy * std::sin(h);
    speeds.left    = -vx * std::sin(h) + vy * std::cos(h);
    speeds.turn    = std::clamp(turn, -max_turn_rate_, max_turn_rate_);
    return speeds;
}

} // namespace navigatr
