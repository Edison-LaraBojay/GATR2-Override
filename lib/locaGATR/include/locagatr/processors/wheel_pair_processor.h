// wheel_pair_processor.h
// Yaw from two forward tracking wheels, measurement "wheel pair". Runs after
// TrackingWheelProcessor.

#pragma once
#include "locagatr/pipeline.h"

namespace locagatr
{

class WheelPairProcessor : public Processor {
  public:
    const char* name() const override { return "wheel pair"; }
    bool        begin(const Config& config) override;
    void        process(const SensorMap& sensors, MotionMap& motion) override;

  private:
    const char* left_         = nullptr;
    const char* right_        = nullptr;
    double      left_offset_  = 0;
    double      right_offset_ = 0;
};

} // namespace locagatr
