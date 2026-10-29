// localizer.cpp
// PROS only.

#include "locagatr/localizer.h"

#include <algorithm>

#include "locagatr/report.h"
#include "pros/rtos.hpp"

namespace locagatr
{

namespace
{

constexpr char kTaskName[] = "locagatr";

} // namespace

bool Localizer::start() {
    int expected = 0;
    if (!started_.compare_exchange_strong(expected, 1)) {
        return expected == 1 && start_ok_;
    }
    const bool begun   = pipeline_.begin(); // sets the start pose too
    const bool task_ok =pros::c::task_create(&Localizer::taskEntry, this, TASK_PRIORITY_DEFAULT + 1,
                                              TASK_STACK_DEPTH_DEFAULT, kTaskName) != nullptr;
    if (!task_ok) {
        started_.store(-1);
    }
    start_ok_ = begun && task_ok;
    return start_ok_;
}

Localizer::Snapshot Localizer::snapshot() const {
    Snapshot s;
    if (!snapshot_.load(s)) {
        s.pose = config_.start_pose;
    }
    return s;
}

Pose   Localizer::getPose() const { return snapshot().pose; }
double Localizer::getX() const { return getPose().x; }
double Localizer::getY() const { return getPose().y; }
double Localizer::getHeading() const { return getPose().heading; }
double Localizer::getPitch() const { return getPose().pitch; }
double Localizer::getRoll() const { return getPose().roll; }

void Localizer::setPose(const Pose& pose) {
    pose_request_.store(pose);
    if (started_.load() != 1) {
        return; // the task takes it when it starts
    }
    const uint32_t version  = pose_request_.version();
    const uint32_t deadline = pros::millis() + 4 * 1000 / std::max<uint32_t>(1, config_.rate_hz) + 5;
    while (static_cast<int32_t>(pose_applied_.load() - version) < 0 &&
           static_cast<int32_t>(deadline - pros::millis()) > 0) {
        pros::delay(1);
    }
}

void Localizer::setPose(double x, double y, double heading) {
    Pose pose;
    pose.x       = x;
    pose.y       = y;
    pose.heading = heading;
    setPose(pose);
}

Status Localizer::getStatus() const { return snapshot().status; }

std::string Localizer::describe() const { return locagatr::describe(config_, getStatus()); }

void Localizer::run() {
    const uint32_t period_ms = std::max<uint32_t>(1, 1000 / std::max<uint32_t>(1, config_.rate_hz));
    uint32_t       seen      = 0; // pose_request_ version applied; 0 = none stored
    Snapshot       s;
    s.status.sources_running    = pipeline_.sourcesRunning();
    s.status.processors_running = pipeline_.processorsRunning();
    s.status.estimator_running  = pipeline_.estimatorRunning();

    uint32_t next = pros::millis();
    while (true) {
        const uint32_t version = pose_request_.version();
        Pose           requested;
        if (version != seen && pose_request_.load(requested)) {
            pipeline_.setPose(requested);
            seen = version;
        }

        const Tick tick = pipeline_.tick(pros::micros());

        s.pose            = tick.estimate.pose;
        s.status.sensors  = tick.sensors;
        s.status.motion   = tick.motion;
        s.status.estimate = tick.estimate;
        ++s.status.ticks;
        if (pros::millis() - next > period_ms) {
            ++s.status.overruns;
        }
        snapshot_.store(s);
        pose_applied_.store(seen);

        pros::c::task_delay_until(&next, period_ms);
    }
}

} // namespace locagatr
