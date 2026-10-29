// pid.h
// PID on an error. Portable.

#pragma once

namespace navigatr
{

class Pid {
  public:
    Pid() = default;
    Pid(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd) {}

    double update(double error, double dt) {
        integral_ += error * dt;
        const double derivative = (first_ || dt <= 0) ? 0.0 : (error - last_) / dt;
        first_                  = false;
        last_                   = error;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

    void reset() {
        integral_ = 0;
        last_     = 0;
        first_    = true;
    }

  private:
    double kp_       = 0;
    double ki_       = 0;
    double kd_       = 0;
    double integral_ = 0;
    double last_     = 0;
    bool   first_    = true;
};

} // namespace navigatr
