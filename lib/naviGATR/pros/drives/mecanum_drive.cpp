// mecanum_drive.cpp
// PROS only: the motors.

#include "navigatr/drives/mecanum_drive.h"

#include <cmath>
#include <cstdlib>
#include <vector>

#include "pros/motors.h"

namespace navigatr
{

namespace
{

const std::vector<int8_t>& corner(const gatr2::DriveMotors& d, std::size_t i) {
    const std::vector<int8_t>* all[4] = {&d.front_left, &d.front_right, &d.back_left, &d.back_right};
    return *all[i];
}

} // namespace

bool MecanumDrive::begin(const Config& config) {
    motors_         = config.robot.drive;
    feedforward_    = config.feedforward;
    feedforward_.kv = wheelKv(motors_, feedforward_);
    for (std::size_t i = 0; i < 4; ++i) {
        if (corner(motors_, i).empty()) {
            return false;
        }
    }
    return feedforward_.max_mv > 0 && feedforward_.kv > 0;
}

void MecanumDrive::apply(const ChassisSpeeds& speeds) {
    const std::array<double, 4> mv = mecanumVoltages(speeds, motors_, feedforward_);
    for (std::size_t i = 0; i < 4; ++i) {
        for (const int8_t port : corner(motors_, i)) {
            const auto out = static_cast<int32_t>(std::lround(port < 0 ? -mv[i] : mv[i]));
            pros::c::motor_move_voltage(static_cast<int8_t>(std::abs(port)), out);
        }
    }
}

void MecanumDrive::stop() {
    for (std::size_t i = 0; i < 4; ++i) {
        for (const int8_t port : corner(motors_, i)) {
            pros::c::motor_move_voltage(static_cast<int8_t>(std::abs(port)), 0);
        }
    }
}

} // namespace navigatr
