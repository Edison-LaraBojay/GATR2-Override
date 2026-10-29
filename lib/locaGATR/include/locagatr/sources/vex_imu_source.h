// vex_imu_source.h
// VEX Inertial Sensor. begin() calibrates, about 2 s, robot still. Source in
// pros/.

#pragma once
#include <cstdint>

#include "locagatr/pipeline.h"

namespace locagatr
{

class VexImuSource : public Source {
  public:
    const char* name() const override { return "VEX IMU"; }
    bool        begin(const Config& config) override;
    void        read(SensorMap& sensors) override;

  private:
    uint8_t port_ = 0;
};

} // namespace locagatr
