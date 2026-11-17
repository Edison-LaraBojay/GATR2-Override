// path_gtest.cpp
// Sampling segments into a path, and reading a trajectory by time.

#include <gtest/gtest.h>

#include <cmath>

#include "navigatr/profiles/profile.h"

namespace navigatr
{
namespace
{

// A cubic Bezier close to a quarter circle of radius r about the origin, from
// (r, 0) to (0, r).
Bezier quarterCircle(double r) {
    const double k = 0.5522847498 * r;
    return {{r, 0}, {r, k}, {k, r}, {0, r}};
}

TEST(Path, LineLengthAndTangent) {
    Segment line;
    line.curve      = Bezier::line({0, 0}, {30, 40});
    const Path path = samplePath({line});
    EXPECT_NEAR(path.length(), 50, 1e-9);
    EXPECT_NEAR(path.points[10].tangent, std::atan2(40.0, 30.0) * 180 / kPi, 1e-9);
    EXPECT_NEAR(path.points[10].curvature, 0, 1e-9);
}

TEST(Path, QuarterCircleLengthAndCurvature) {
    Segment arc;
    arc.curve       = quarterCircle(24);
    const Path path = samplePath({arc}, 200);
    EXPECT_NEAR(path.length(), 24 * kPi / 2, 0.01);
    EXPECT_NEAR(path.points[100].curvature, 1.0 / 24, 0.001); // counterclockwise
    EXPECT_NEAR(path.points[100].tangent, 135, 0.5);
}

TEST(Path, SegmentsJoinWithoutRepeatingThePoint) {
    Segment a, b;
    a.curve         = Bezier::line({0, 0}, {10, 0});
    b.curve         = Bezier::line({10, 0}, {10, 10});
    const Path path = samplePath({a, b}, 10);
    EXPECT_EQ(path.points.size(), 21u);
    EXPECT_NEAR(path.length(), 20, 1e-9);
}

TEST(Path, HeadingTurnsEvenlyTheShortWay) {
    Segment s;
    s.curve         = Bezier::line({0, 0}, {10, 0});
    s.start_heading = 350;
    s.end_heading   = 10;
    const Path path = samplePath({s}, 10);
    EXPECT_NEAR(path.points[5].heading, 360, 1e-9); // through 0, not back through 180
    EXPECT_NEAR(path.points.back().heading, 370, 1e-9);
}

TEST(Path, TangentHeadingFollowsThePath) {
    Segment arc;
    arc.curve       = quarterCircle(24);
    arc.tangent     = true;
    const Path path = samplePath({arc}, 100);
    EXPECT_NEAR(path.points[50].heading, path.points[50].tangent, 1e-9);
}

TEST(Path, MarkersByDistance) {
    Segment a, b;
    a.curve = Bezier::line({0, 0}, {10, 0});
    b.curve = Bezier::line({10, 0}, {30, 0});
    b.actions.push_back({0.5, [] {}});
    a.actions.push_back({1.0, [] {}});
    const Path path = samplePath({a, b});
    ASSERT_EQ(path.markers.size(), 2u);
    EXPECT_NEAR(path.markers[0].s, 10, 1e-9);
    EXPECT_NEAR(path.markers[1].s, 20, 1e-9);
}

TEST(Trajectory, InterpolatesAndStopsAtTheEnd) {
    Trajectory trajectory;
    trajectory.points = {{0, 0, 0, 0, 0, 0, 0, 10}, {1, 10, 10, 0, 20, 0, 10, 10}};
    const TrajectoryPoint mid = trajectory.at(0.5);
    EXPECT_DOUBLE_EQ(mid.x, 5);
    EXPECT_DOUBLE_EQ(mid.v, 10);
    EXPECT_DOUBLE_EQ(mid.heading, 5);
    const TrajectoryPoint after = trajectory.at(3);
    EXPECT_DOUBLE_EQ(after.x, 10);
    EXPECT_DOUBLE_EQ(after.v, 0);
    EXPECT_DOUBLE_EQ(after.heading_rate, 0);
}

} // namespace
} // namespace navigatr
