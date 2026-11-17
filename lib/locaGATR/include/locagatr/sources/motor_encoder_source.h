// motor_encoder_source.h
// Drive motor positions from raw encoder counts, by port. Source in pros/.

#pragma once
#include <cstdint>
#include <vector>

#include "locagatr/sources/source.h"

namespace locagatr
{

class MotorEncoderSource : public Source {
  public:
    const char* name() const override { return "motor encoders"; }
    bool        begin(const Config& config) override;
    void        read(SensorMap& sensors) override;

  private:
    std::vector<uint8_t> ports_;
};

} // namespace locagatr
