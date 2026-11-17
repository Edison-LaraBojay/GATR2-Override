// path.h
// Stage 1, path: a move's segments -> Path, evenly sampled. Portable.

#pragma once
#include <functional>
#include <vector>

#include "navigatr/route.h"
#include "navigatr/types.h"

namespace navigatr
{

struct PathPoint {
    double x         = 0; // in
    double y         = 0; // in
    double s         = 0; // in along the path
    double tangent   = 0; // deg, the way the path goes
    double curvature = 0; // 1/in, counterclockwise positive
    double heading   = 0; // deg, the way the robot should face; continuous
};

struct Path {
    struct Marker {
        double                       s = 0; // in along the path
        const std::function<void()>* run = nullptr;
    };
    std::vector<PathPoint> points;
    std::vector<Marker>    markers; // in order of s

    double length() const { return points.empty() ? 0.0 : points.back().s; }
};

// One move, sampled. Markers point into the segments.
Path samplePath(const std::vector<Segment>& segments, int samples_per_segment = 64);

} // namespace navigatr
