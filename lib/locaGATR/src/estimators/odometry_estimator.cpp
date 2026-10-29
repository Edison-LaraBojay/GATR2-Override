// odometry_estimator.cpp

#include "locagatr/estimators/odometry_estimator.h"

#include <cmath>

namespace locagatr
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double radians(double degrees) { return degrees * kPi / 180.0; }
double degrees(double radians) { return radians * 180.0 / kPi; }

} // namespace

bool OdometryEstimator::begin(const Config& config) {
    sources_ = config.odometry;
    return sources_.forward.size() <= kMaxMeasurements &&
           (!sources_.forward.empty() || sources_.lateral != nullptr || sources_.yaw != nullptr);
}

// The chosen forward and lateral totals, or nullopt and the missing name.
std::optional<OdometryEstimator::Totals> OdometryEstimator::read(const MotionMap& motion,
                                                                 const char*&     missing) const {
    Totals t;
    for (std::size_t i = 0; i < sources_.forward.size(); ++i) {
        const Measurement* m = motion.find(sources_.forward[i]);
        if (m == nullptr || !m->forward) {
            missing = sources_.forward[i];
            return std::nullopt;
        }
        t.forward[i]        = *m->forward;
        t.forward_offset[i] = m->offset;
    }
    if (sources_.lateral != nullptr) {
        const Measurement* m = motion.find(sources_.lateral);
        if (m == nullptr || !m->lateral) {
            missing = sources_.lateral;
            return std::nullopt;
        }
        t.lateral        = *m->lateral;
        t.lateral_offset = m->offset;
    }
    return t;
}

Estimate OdometryEstimator::update(const MotionMap& motion, const SensorMap&) {
    // Heading. The first yaw seen is the baseline.
    Estimate           e;
    const Measurement* yaw = motion.find(sources_.yaw);
    if (sources_.yaw != nullptr && (yaw == nullptr || !yaw->yaw)) {
        e.missing = sources_.yaw;
    }
    if (yaw != nullptr && yaw->yaw) {
        const double total = radians(*yaw->yaw);
        if (!have_yaw_) {
            yaw_offset_ = heading_ - total;
            have_yaw_   = true;
        }
        yaw_     = total;
        heading_ = total + yaw_offset_;
    }
    if (const Measurement* attitude = motion.find(sources_.attitude)) {
        pitch_ = attitude->pitch.value_or(pitch_);
        roll_  = attitude->roll.value_or(roll_);
    }

    // Translation.
    const std::optional<Totals> now = read(motion, e.missing);
    if (now) {
        if (last_) {
            // Off-center wheels roll on a turn: left of center backward, ahead of
            // center left.
            const double      turn    = heading_ - advanced_at_;
            const std::size_t n       = sources_.forward.size();
            double            forward = 0;
            for (std::size_t i = 0; i < n; ++i) {
                forward += now->forward[i] - last_->forward[i] + now->forward_offset[i] * turn;
            }
            forward = n > 0 ? forward / n : 0.0;
            const double left =
                sources_.lateral != nullptr
                    ? now->lateral - last_->lateral - now->lateral_offset * turn
                    : 0.0;

            e.motion = BodyMotion{forward, left, degrees(turn)};
            Pose from;
            from.x        = x_;
            from.y        = y_;
            from.heading  = degrees(advanced_at_);
            const Pose to = advance(from, e.motion);
            x_            = to.x;
            y_            = to.y;
        }
        last_        = now;
        advanced_at_ = heading_;
        e.tracking   = true;
    }

    double heading = std::fmod(degrees(heading_), 360.0);
    if (heading < 0) {
        heading += 360.0;
    }
    e.pose = Pose{x_, y_, heading, pitch_, roll_};
    return e;
}

void OdometryEstimator::setPose(const Pose& pose) {
    x_           = pose.x;
    y_           = pose.y;
    heading_     = radians(pose.heading);
    advanced_at_ = heading_;
    if (have_yaw_) {
        yaw_offset_ = heading_ - yaw_;
    }
}

} // namespace locagatr
