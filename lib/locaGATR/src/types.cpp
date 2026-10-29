// types.cpp

#include "locagatr/types.h"

#include <cmath>

namespace locagatr
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

} // namespace

Pose advance(const Pose& pose, const BodyMotion& motion) {
    const double half  = motion.turn * kPi / 360.0; // rad
    const double chord = std::fabs(half) < 1e-6 ? 1.0 - half * half / 6.0 : std::sin(half) / half;
    const double mid   = pose.heading * kPi / 180.0 + half;

    Pose next    = pose;
    next.x       = pose.x + chord * (motion.forward * std::cos(mid) - motion.left * std::sin(mid));
    next.y       = pose.y + chord * (motion.forward * std::sin(mid) + motion.left * std::cos(mid));
    next.heading = std::fmod(pose.heading + motion.turn, 360.0);
    if (next.heading < 0) {
        next.heading += 360.0;
    }
    return next;
}

} // namespace locagatr
