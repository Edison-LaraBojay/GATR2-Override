// drive_motor_processor.cpp

#include "locagatr/processors/drive_motor_processor.h"

#include <cstdlib>

namespace locagatr
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

} // namespace

bool DriveMotorProcessor::begin(const Config& config) {
    const DriveMotors& d = config.robot.drive;
    corners_             = {d.front_left, d.front_right, d.back_left, d.back_right};
    mecanum_             = d.mecanum;
    for (const std::vector<int8_t>& corner : corners_) {
        for (const int8_t port : corner) {
            if (std::abs(port) < 1 || std::abs(port) > 21) {
                return false;
            }
        }
        if (mecanum_ && corner.empty()) {
            return false;
        }
    }
    const bool left    = !d.front_left.empty() || !d.back_left.empty();
    const bool right   = !d.front_right.empty() || !d.back_right.empty();
    inches_per_degree_ = d.gear_ratio * kPi * d.wheel_diameter / 360.0;
    return left && right && inches_per_degree_ > 0;
}

// A turn in place cancels in the left/right mean. Mecanum strafe left: front
// left and back right turn backward.
void DriveMotorProcessor::process(const SensorMap& sensors, MotionMap& motion) {
    // Each corner: mean of its motors, in inches.
    std::array<double, kCorners> wheel{};
    for (std::size_t c = 0; c < kCorners; ++c) {
        double sum = 0;
        for (const int8_t signed_port : corners_[c]) {
            const std::optional<double>& degrees = sensors.motor_positions[std::abs(signed_port)];
            if (!degrees) {
                return;
            }
            sum += signed_port < 0 ? -*degrees : *degrees;
        }
        wheel[c] = corners_[c].empty() ? 0.0 : sum / corners_[c].size() * inches_per_degree_;
    }
    const auto side = [&](Corner front, Corner back) {
        const int n = (corners_[front].empty() ? 0 : 1) + (corners_[back].empty() ? 0 : 1);
        return (wheel[front] + wheel[back]) / n;
    };

    Measurement* m = motion.add("drive");
    if (m == nullptr) {
        return;
    }
    m->forward = (side(kFrontLeft, kBackLeft) + side(kFrontRight, kBackRight)) / 2.0;
    if (mecanum_) {
        m->lateral =
            (-wheel[kFrontLeft] + wheel[kFrontRight] + wheel[kBackLeft] - wheel[kBackRight]) / 4.0;
    }
}

} // namespace locagatr
