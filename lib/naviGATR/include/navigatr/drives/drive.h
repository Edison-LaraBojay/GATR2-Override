// drive.h
// Stage 4, drive: command -> motors, every tick. Portable.

#pragma once
#include "navigatr/config.h"
#include "navigatr/types.h"

namespace navigatr
{

class Drive {
  public:
    virtual ~Drive() = default;
    virtual const char* name() const = 0;
    virtual bool        begin(const Config&) { return true; }
    virtual void        apply(const ChassisSpeeds& speeds) = 0;
    virtual void        stop() = 0;
};

} // namespace navigatr
