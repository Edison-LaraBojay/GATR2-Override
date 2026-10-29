// types.h
// Pose and motion. Inches and degrees. Field: +x right, +y up, heading
// counterclockwise from +x. Robot: +x forward, +y left. Portable.

#pragma once

namespace locagatr
{

struct Pose {
    double x       = 0; // in
    double y       = 0; // in
    double heading = 0; // deg, [0, 360)
    double pitch   = 0; // deg, from the IMU
    double roll    = 0; // deg, from the IMU
};

struct Vector3 {
    double x = 0;
    double y = 0;
    double z = 0;
};

// Robot frame motion over one tick, along the arc.
struct BodyMotion {
    double forward = 0; // in
    double left    = 0; // in
    double turn    = 0; // deg counterclockwise
};

// The pose after this motion, along a constant curvature arc.
Pose advance(const Pose& pose, const BodyMotion& motion);

} // namespace locagatr
