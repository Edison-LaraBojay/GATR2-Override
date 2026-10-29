// drive_motor_processor.h
// Drive motors to measurement "drive". Lateral only for mecanum, and it reads
// short: the rollers slip.

#pragma once
#include <array>
#include <vector>

#include "locagatr/pipeline.h"

namespace locagatr
{

class DriveMotorProcessor : public Processor {
  public:
    const char* name() const override { return "drive"; }
    bool        begin(const Config& config) override;
    void        process(const SensorMap& sensors, MotionMap& motion) override;

  private:
    enum Corner { kFrontLeft, kFrontRight, kBackLeft, kBackRight, kCorners };

    std::array<std::vector<int8_t>, kCorners> corners_;
    double                                    inches_per_degree_ = 0;
    bool                                      mecanum_           = false;
};

} // namespace locagatr
