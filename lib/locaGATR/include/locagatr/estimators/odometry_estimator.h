// odometry_estimator.h
// Dead reckoning, one measurement per quantity as chosen in Config::odometry.
// Waits for every chosen forward and lateral measurement, then takes the whole
// change.

#pragma once
#include <array>

#include "locagatr/pipeline.h"

namespace locagatr
{

class OdometryEstimator : public Estimator {
  public:
    const char* name() const override { return "odometry"; }
    bool        begin(const Config& config) override;
    Estimate    update(const MotionMap& motion, const SensorMap& sensors) override;
    void        setPose(const Pose& pose) override;

  private:
    struct Totals {
        std::array<double, kMaxMeasurements> forward{};
        std::array<double, kMaxMeasurements> forward_offset{};
        double                               lateral        = 0;
        double                               lateral_offset = 0;
    };

    std::optional<Totals> read(const MotionMap& motion, const char*& missing) const;

    OdometrySources       sources_;
    std::optional<Totals> last_;
    double                x_           = 0;
    double                y_           = 0;
    double                heading_     = 0; // rad, unwrapped
    double                advanced_at_ = 0; // rad, heading_ at the last move
    double                yaw_offset_  = 0; // rad, heading_ = yaw total + this
    double                yaw_         = 0; // rad, last yaw total
    bool                  have_yaw_    = false;
    double                pitch_       = 0;
    double                roll_        = 0;
};

} // namespace locagatr
