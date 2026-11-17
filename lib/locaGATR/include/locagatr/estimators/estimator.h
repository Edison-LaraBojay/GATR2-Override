// estimator.h
// Stage 3, estimation: Estimator -> Estimate, the published pose. Portable.

#pragma once
#include "locagatr/config.h"
#include "locagatr/processors/processor.h"
#include "locagatr/sources/source.h"
#include "locagatr/types.h"

namespace locagatr
{

struct Estimate {
    Pose       pose;
    BodyMotion motion;           // this tick
    bool       tracking = false; // had every measurement it needs this tick
    // The chosen measurement it did not get this tick, by name.
    const char* missing = nullptr;
};

class Estimator {
  public:
    virtual ~Estimator() = default;
    virtual const char* name() const = 0;
    virtual bool        begin(const Config&) { return true; }
    virtual Estimate    update(const MotionMap& motion, const SensorMap& sensors) = 0;
    virtual void        setPose(const Pose& pose) = 0;
};

} // namespace locagatr
