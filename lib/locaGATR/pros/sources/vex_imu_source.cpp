// vex_imu_source.cpp
// PROS only.

#include "locagatr/sources/vex_imu_source.h"

#include <cmath>

#include "pros/error.h"
#include "pros/imu.h"

namespace locagatr
{

namespace
{

bool finite(const pros::imu_raw_s& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

} // namespace

bool VexImuSource::begin(const Config& config) {
    port_ = config.robot.imu_port;
    return port_ >= 1 && port_ <= 21 && pros::c::imu_reset_blocking(port_) != PROS_ERR;
}

void VexImuSource::read(SensorMap& sensors) {
    // PROS_ERR_F (infinity) on failure or while calibrating.
    const double          rotation = pros::c::imu_get_rotation(port_);
    const double          pitch    = pros::c::imu_get_pitch(port_);
    const double          roll     = pros::c::imu_get_roll(port_);
    const pros::imu_raw_s gyro     = pros::c::imu_get_gyro_rate(port_);
    const pros::imu_raw_s accel    = pros::c::imu_get_accel(port_);
    if (!std::isfinite(rotation) || !std::isfinite(pitch) || !std::isfinite(roll) ||
        !finite(gyro) || !finite(accel)) {
        return;
    }
    ImuSample s;
    s.rotation  = -rotation; // PROS rotation is clockwise
    s.pitch     = pitch;
    s.roll      = roll;
    s.gyro      = Vector3{gyro.x, gyro.y, gyro.z};
    s.accel     = Vector3{accel.x, accel.y, accel.z};
    sensors.imu = s;
}

} // namespace locagatr
