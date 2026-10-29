// types.h
// Points, speeds, angles. Inches and degrees, the same frames as locaGATR.
// Portable.

#pragma once
#include <cmath>

#include "locagatr/types.h"

namespace navigatr
{

using Pose = locagatr::Pose;

struct Point {
    double x = 0; // in
    double y = 0; // in
};

inline Point operator+(Point a, Point b) { return {a.x + b.x, a.y + b.y}; }
inline Point operator-(Point a, Point b) { return {a.x - b.x, a.y - b.y}; }
inline Point operator*(double k, Point p) { return {k * p.x, k * p.y}; }

// Robot frame command.
struct ChassisSpeeds {
    double forward = 0; // in/s
    double left    = 0; // in/s
    double turn    = 0; // deg/s counterclockwise
};

constexpr double kPi = 3.14159265358979323846;

inline double radians(double degrees) { return degrees * kPi / 180.0; }
inline double degrees(double radians) { return radians * 180.0 / kPi; }

// [-180, 180).
inline double wrap180(double deg) {
    const double wrapped = std::fmod(deg + 180.0, 360.0);
    return (wrapped < 0 ? wrapped + 360.0 : wrapped) - 180.0;
}

} // namespace navigatr
