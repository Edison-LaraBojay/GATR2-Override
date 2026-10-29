// bezier.h
// A cubic Bezier: start, two control points, end. Every route segment is one.
// Portable.

#pragma once
#include "navigatr/types.h"

namespace navigatr
{

struct Bezier {
    Point p0;
    Point c1;
    Point c2;
    Point p1;

    static Bezier line(Point from, Point to) {
        return {from, from + (1.0 / 3.0) * (to - from), from + (2.0 / 3.0) * (to - from), to};
    }

    // t in [0, 1].
    Point at(double t) const {
        const double u = 1 - t;
        return (u * u * u) * p0 + (3 * u * u * t) * c1 + (3 * u * t * t) * c2 + (t * t * t) * p1;
    }

    // First derivative by t.
    Point velocity(double t) const {
        const double u = 1 - t;
        return (3 * u * u) * (c1 - p0) + (6 * u * t) * (c2 - c1) + (3 * t * t) * (p1 - c2);
    }

    // Second derivative by t.
    Point acceleration(double t) const {
        return (6 * (1 - t)) * (c2 - 2.0 * c1 + p0) + (6 * t) * (p1 - 2.0 * c2 + c1);
    }
};

} // namespace navigatr
