// navigator.cpp
// PROS only.

#include "navigatr/navigator.h"

#include "pros/misc.h"
#include "pros/rtos.hpp"

namespace navigatr
{

namespace
{

constexpr char     kTaskName[] = "navigatr";
constexpr uint32_t kPollMs     = 5;

// Disabled and autonomous bits of the competition status.
int competitionMode() { return pros::c::competition_get_status() & 0x03; }

} // namespace

Navigator::Navigator(const Config& config, const locagatr::Localizer& localizer)
    : pipeline_(config, [this, &localizer] {
          Io io;
          io.pose        = [&localizer] { return localizer.getPose(); };
          io.now_ms      = [] { return pros::millis(); };
          io.sleep_until = [](uint32_t ms) {
              const uint32_t now = pros::millis();
              if (static_cast<int32_t>(ms - now) > 0) {
                  pros::delay(ms - now);
              }
          };
          io.cancelled = [this] { return cancel_.load() || competitionMode() != mode_.load(); };
          io.progress  = [this](double fraction) { progress_.store(static_cast<float>(fraction)); };
          return io;
      }()) {}

bool Navigator::start(const Route& route) {
    if (running_.load()) {
        return false;
    }
    if (begun_ == 0) {
        begun_ = pipeline_.begin() ? 1 : -1;
    }
    if (begun_ != 1) {
        return false;
    }
    if (task_ == nullptr) {
        task_ = pros::c::task_create(&Navigator::taskEntry, this, TASK_PRIORITY_DEFAULT,
                                     TASK_STACK_DEPTH_DEFAULT, kTaskName);
        if (task_ == nullptr) {
            return false;
        }
    }
    route_   = route; // the task is asleep: running_ is false
    outcome_ = Outcome{};
    cancel_.store(false);
    progress_.store(0);
    mode_.store(competitionMode());
    running_.store(true);
    pros::c::task_notify(static_cast<pros::task_t>(task_));
    return true;
}

void Navigator::run() {
    while (true) {
        pros::c::task_notify_take(true, TIMEOUT_MAX);
        if (!running_.load()) {
            continue;
        }
        outcome_ = pipeline_.run(route_);
        running_.store(false);
    }
}

bool Navigator::wait() {
    while (running_.load()) {
        pros::delay(kPollMs);
    }
    return outcome_.ok();
}

bool Navigator::waitUntil(double fraction) {
    while (running_.load() && progress_.load() < fraction) {
        pros::delay(kPollMs);
    }
    return progress_.load() >= fraction;
}

bool Navigator::follow(const Route& route) { return start(route) && wait(); }

void Navigator::cancel() {
    cancel_.store(true);
    while (running_.load()) {
        pros::delay(1);
    }
}

} // namespace navigatr
