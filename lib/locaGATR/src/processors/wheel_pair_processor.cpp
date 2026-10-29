// wheel_pair_processor.cpp

#include "locagatr/processors/wheel_pair_processor.h"

#include <cstring>

namespace locagatr
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

const TrackingWheel* forwardWheel(const Config& config, const char* name) {
    for (const TrackingWheel& w : config.robot.tracking_wheels) {
        if (name != nullptr && std::strcmp(w.name, name) == 0 && w.axis == Axis::kForward) {
            return &w;
        }
    }
    return nullptr;
}

} // namespace

bool WheelPairProcessor::begin(const Config& config) {
    const TrackingWheel* left  = forwardWheel(config, config.robot.wheel_pair.left);
    const TrackingWheel* right = forwardWheel(config, config.robot.wheel_pair.right);
    if (left == nullptr || right == nullptr || left->offset == right->offset) {
        return false;
    }
    left_         = left->name;
    right_        = right->name;
    left_offset_  = left->offset;
    right_offset_ = right->offset;
    return true;
}

// A forward wheel rolls (forward - offset * turn), so the difference of two is
// (right offset - left offset) * turn.
void WheelPairProcessor::process(const SensorMap&, MotionMap& motion) {
    const Measurement* left  = motion.find(left_);
    const Measurement* right = motion.find(right_);
    if (left == nullptr || right == nullptr || !left->forward || !right->forward) {
        return;
    }
    const double turn = (*left->forward - *right->forward) / (right_offset_ - left_offset_);
    if (Measurement* m = motion.add("wheel pair")) {
        m->yaw = turn * 180.0 / kPi;
    }
}

} // namespace locagatr
