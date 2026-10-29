// motor_encoder_source.cpp
// PROS only.

#include "locagatr/sources/motor_encoder_source.h"

#include <algorithm>
#include <cstdlib>

#include "pros/error.h"
#include "pros/motors.h"

namespace locagatr
{

namespace
{

// Raw counts per output shaft turn, by cartridge.
double countsPerTurn(pros::motor_gearset_e_t gearing) {
    switch (gearing) {
    case pros::E_MOTOR_GEARSET_36: return 1800;
    case pros::E_MOTOR_GEARSET_18: return 900;
    case pros::E_MOTOR_GEARSET_06: return 300;
    default: return 0;
    }
}

} // namespace

bool MotorEncoderSource::begin(const Config& config) {
    ports_.clear();
    const DriveMotors& d = config.robot.drive;
    for (const std::vector<int8_t>* corner : {&d.front_left, &d.front_right, &d.back_left, &d.back_right}) {
        for (const int8_t signed_port : *corner) {
            const auto port = static_cast<uint8_t>(std::abs(signed_port));
            if (port < 1 || port > 21) {
                return false;
            }
            if (std::find(ports_.begin(), ports_.end(), port) == ports_.end()) {
                ports_.push_back(port);
            }
        }
    }
    return !ports_.empty();
}

void MotorEncoderSource::read(SensorMap& sensors) {
    for (const uint8_t port : ports_) {
        const double per_turn = countsPerTurn(pros::c::motor_get_gearing(port));
        const int32_t counts  = pros::c::motor_get_raw_position(port, nullptr);
        if (per_turn > 0 && counts != PROS_ERR) {
            sensors.motor_positions[port] = counts * 360.0 / per_turn;
        }
    }
}

} // namespace locagatr
