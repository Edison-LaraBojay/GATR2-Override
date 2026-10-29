// trapezoid_profile.h
// Accelerate, cruise, decelerate, rest to rest. Slower in tight curves and
// fast heading changes.

#pragma once
#include "navigatr/pipeline.h"

namespace navigatr
{

class TrapezoidProfile : public Profile {
  public:
    const char* name() const override { return "trapezoid"; }
    bool        begin(const Config& config) override;
    Trajectory  build(const Path& path) const override;

  private:
    Limits limits_;
};

} // namespace navigatr
