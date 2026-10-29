// imu_processor.h
// IMU sample to measurement "IMU".

#pragma once
#include "locagatr/pipeline.h"

namespace locagatr
{

class ImuProcessor : public Processor {
  public:
    const char* name() const override { return "IMU"; }

    void process(const SensorMap& sensors, MotionMap& motion) override {
        if (!sensors.imu) {
            return;
        }
        if (Measurement* m = motion.add("IMU")) {
            m->yaw   = sensors.imu->rotation;
            m->pitch = sensors.imu->pitch;
            m->roll  = sensors.imu->roll;
        }
    }
};

} // namespace locagatr
