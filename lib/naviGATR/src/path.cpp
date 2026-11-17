// path.cpp
// Stage 1: segments to a Path.

#include <algorithm>
#include <cmath>

#include "navigatr/path.h"

namespace navigatr
{

namespace
{

struct Range {
    std::size_t first = 0; // this segment's points: [first, last]
    std::size_t last  = 0;
    double      s0    = 0; // in along the path where the segment starts
};

} // namespace

Path samplePath(const std::vector<Segment>& segments, int samples_per_segment) {
    const int          n = std::max(2, samples_per_segment);
    Path               path;
    std::vector<Range> ranges;

    // Points, distance along, curvature. A segment's first point is the last
    // one's end, so it is skipped.
    for (std::size_t i = 0; i < segments.size(); ++i) {
        const Bezier& curve = segments[i].curve;
        Range         range;
        range.first = path.points.size();
        range.s0    = path.points.empty() ? 0.0 : path.points.back().s;
        for (int k = path.points.empty() ? 0 : 1; k <= n; ++k) {
            const double t = static_cast<double>(k) / n;
            const Point  p = curve.at(t);
            PathPoint    point;
            point.x = p.x;
            point.y = p.y;
            point.s = path.points.empty() ? 0.0
                                          : path.points.back().s + std::hypot(p.x - path.points.back().x,
                                                                              p.y - path.points.back().y);
            const Point  v     = curve.velocity(t);
            const Point  a     = curve.acceleration(t);
            const double speed = std::hypot(v.x, v.y);
            point.curvature    = speed > 1e-9 ? (v.x * a.y - v.y * a.x) / (speed * speed * speed) : 0.0;
            path.points.push_back(point);
        }
        range.last = path.points.size() - 1;
        ranges.push_back(range);
    }

    // Tangent from the neighbors.
    double tangent = segments.empty() ? 0.0 : segments.front().start_heading;
    for (std::size_t i = 0; i < path.points.size(); ++i) {
        const PathPoint& before = path.points[i == 0 ? 0 : i - 1];
        const PathPoint& after  = path.points[std::min(i + 1, path.points.size() - 1)];
        if (std::hypot(after.x - before.x, after.y - before.y) > 1e-9) {
            tangent = degrees(std::atan2(after.y - before.y, after.x - before.x));
        }
        path.points[i].tangent = tangent;
    }

    // Heading: the tangent, or start to end heading by distance. Continuous.
    double heading = segments.empty() ? 0.0 : segments.front().start_heading;
    for (std::size_t i = 0; i < segments.size(); ++i) {
        const Segment& segment = segments[i];
        const Range&   range   = ranges[i];
        const double   length  = path.points[range.last].s - range.s0;
        for (std::size_t p = range.first; p <= range.last; ++p) {
            PathPoint&   point = path.points[p];
            const double f     = length > 1e-9 ? (point.s - range.s0) / length : 1.0;
            const double want =
                segment.tangent
                    ? point.tangent
                    : segment.start_heading +
                          wrap180(segment.end_heading - segment.start_heading) * f;
            heading += wrap180(want - heading);
            point.heading = heading;
        }
        for (const Action& action : segment.actions) {
            path.markers.push_back({range.s0 + std::clamp(action.at, 0.0, 1.0) * length, &action.run});
        }
    }
    std::stable_sort(path.markers.begin(), path.markers.end(),
                     [](const Path::Marker& a, const Path::Marker& b) { return a.s < b.s; });
    return path;
}

} // namespace navigatr
