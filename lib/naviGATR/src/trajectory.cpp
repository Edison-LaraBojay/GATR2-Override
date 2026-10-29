// trajectory.cpp
// Reading a Trajectory by time.

#include <algorithm>

#include "navigatr/pipeline.h"

namespace navigatr
{

namespace
{

double lerp(double a, double b, double f) { return a + (b - a) * f; }

} // namespace

TrajectoryPoint Trajectory::at(double t) const {
    if (points.empty()) {
        return {};
    }
    if (t <= points.front().t) {
        return points.front();
    }
    if (t >= duration()) {
        TrajectoryPoint end = points.back();
        end.v               = 0;
        end.heading_rate    = 0;
        return end;
    }
    const auto after = std::upper_bound(points.begin(), points.end(), t,
                                        [](double time, const TrajectoryPoint& p) { return time < p.t; });
    const TrajectoryPoint& b = *after;
    const TrajectoryPoint& a = *(after - 1);
    const double           f = b.t > a.t ? (t - a.t) / (b.t - a.t) : 0.0;

    TrajectoryPoint p;
    p.t            = t;
    p.s            = lerp(a.s, b.s, f);
    p.x            = lerp(a.x, b.x, f);
    p.y            = lerp(a.y, b.y, f);
    p.v            = lerp(a.v, b.v, f);
    p.tangent      = a.tangent + wrap180(b.tangent - a.tangent) * f;
    p.heading      = lerp(a.heading, b.heading, f);
    p.heading_rate = a.heading_rate;
    return p;
}

} // namespace navigatr
