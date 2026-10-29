// report.h
// The Localizer state as text, one line per stage. Portable.
//
//   sources     VEX IMU: ok | control board: FAILED | motor encoders: off
//   estimator   odometry: waiting for "back"
//   pose        x 12.30 y 0.40 heading 91.2

#pragma once
#include <string>

#include "locagatr/localizer.h"

namespace locagatr
{

std::string describe(const Config& config, const Status& status);

} // namespace locagatr
