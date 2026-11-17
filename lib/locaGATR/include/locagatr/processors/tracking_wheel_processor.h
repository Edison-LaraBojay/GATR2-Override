// tracking_wheel_processor.h
// Encoder counts to inches, one measurement per tracking wheel, under its
// name. Totals carry on across a board restart.

#pragma once
#include <vector>

#include "locagatr/processors/processor.h"

namespace locagatr
{

class TrackingWheelProcessor : public Processor {
  public:
    const char* name() const override { return "tracking wheels"; }
    bool        begin(const Config& config) override;
    void        process(const SensorMap& sensors, MotionMap& motion) override;

  private:
    struct Wheel {
        TrackingWheel config;
        double        base       = 0;     // in, added to the rolled count
        double        last_total = 0;     // in
        bool          rebase     = false; // restarted, no count since
    };

    std::vector<Wheel> wheels_;
};

} // namespace locagatr
