// tracking_wheel_processor.cpp

#include "locagatr/processors/tracking_wheel_processor.h"

#include <cstring>

namespace locagatr
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double rolled(const TrackingWheel& wheel, int32_t counts) {
    return wheel.direction * (counts / wheel.counts_per_rev) * kPi * wheel.diameter;
}

} // namespace

bool TrackingWheelProcessor::begin(const Config& config) {
    wheels_.clear();
    for (const TrackingWheel& w : config.robot.tracking_wheels) {
        if (w.port >= kBoardPorts || w.counts_per_rev <= 0 || w.name == nullptr ||
            std::strlen(w.name) == 0) {
            return false;
        }
        Wheel wheel;
        wheel.config = w;
        wheels_.push_back(wheel);
    }
    return !wheels_.empty();
}

void TrackingWheelProcessor::process(const SensorMap& sensors, MotionMap& motion) {
    for (Wheel& wheel : wheels_) {
        if (sensors.board.restarted) {
            wheel.rebase = true;
        }
        const std::optional<int32_t> count = sensors.encoder_counts[wheel.config.port];
        if (!count) {
            continue;
        }
        const double now = rolled(wheel.config, *count);
        if (wheel.rebase) {
            wheel.base   = wheel.last_total - now; // movement while restarting is lost
            wheel.rebase = false;
        }
        wheel.last_total = wheel.base + now;

        Measurement* m = motion.add(wheel.config.name);
        if (m == nullptr) {
            return;
        }
        if (wheel.config.axis == Axis::kForward) {
            m->forward = wheel.last_total;
        } else {
            m->lateral = wheel.last_total;
        }
        m->offset = wheel.config.offset;
    }
}

} // namespace locagatr
