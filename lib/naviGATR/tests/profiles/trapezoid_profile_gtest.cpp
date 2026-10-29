// trapezoid_profile_gtest.cpp

#include "navigatr/profiles/trapezoid_profile.h"

#include <gtest/gtest.h>

#include <cmath>

namespace navigatr
{
namespace
{

Config limits(double speed, double accel) {
    Config c;
    c.limits.max_speed         = speed;
    c.limits.max_accel         = accel;
    c.limits.max_decel         = accel;
    c.limits.max_lateral_accel = 60;
    c.limits.max_turn_rate     = 180;
    return c;
}

Path straight(double length) {
    Segment line;
    line.curve = Bezier::line({0, 0}, {length, 0});
    return samplePath({line}, 400);
}

TEST(TrapezoidProfile, RestToCruiseToRest) {
    TrapezoidProfile profile;
    ASSERT_TRUE(profile.begin(limits(48, 60)));
    const Trajectory t = profile.build(straight(96));

    EXPECT_DOUBLE_EQ(t.points.front().v, 0);
    EXPECT_DOUBLE_EQ(t.points.back().v, 0);
    double fastest = 0;
    for (const TrajectoryPoint& p : t.points) {
        fastest = std::max(fastest, p.v);
    }
    EXPECT_NEAR(fastest, 48, 1e-6);
    // 0.8 s up over 19.2 in, 57.6 in at 48 in/s (1.2 s), 0.8 s down: 2.8 s.
    EXPECT_NEAR(t.duration(), 2.8, 0.01);
}

TEST(TrapezoidProfile, ShortMoveNeverReachesCruise) {
    TrapezoidProfile profile;
    ASSERT_TRUE(profile.begin(limits(48, 60)));
    const Trajectory t = profile.build(straight(15));
    double           fastest = 0;
    for (const TrajectoryPoint& p : t.points) {
        fastest = std::max(fastest, p.v);
    }
    EXPECT_NEAR(fastest, std::sqrt(60 * 15.0), 0.5); // peak at the middle
    EXPECT_NEAR(t.duration(), 2 * std::sqrt(15.0 / 60), 0.01);
}

TEST(TrapezoidProfile, SlowsInTightCurves) {
    TrapezoidProfile profile;
    ASSERT_TRUE(profile.begin(limits(100, 1000)));
    Segment arc;
    const double r = 6;
    const double k = 0.5522847498 * r;
    arc.curve      = {{r, 0}, {r, k}, {k, r}, {0, r}};
    const Trajectory t = profile.build(samplePath({arc}, 200));
    EXPECT_NEAR(t.points[100].v, std::sqrt(60 * r), 0.2); // lateral accel 60 in/s^2
}

TEST(TrapezoidProfile, SlowsWhenTheHeadingMustTurnFast) {
    TrapezoidProfile profile;
    ASSERT_TRUE(profile.begin(limits(100, 1000)));
    Segment line;
    line.curve         = Bezier::line({0, 0}, {10, 0});
    line.end_heading   = 90; // 90 deg over 10 in
    const Trajectory t = profile.build(samplePath({line}, 100));
    for (const TrajectoryPoint& p : t.points) {
        EXPECT_LE(std::fabs(p.heading_rate), 180 + 1e-6);
    }
    EXPECT_NEAR(t.points[50].v, 180 * 10.0 / 90, 0.01);
}

TEST(TrapezoidProfile, NeedsLimits) {
    TrapezoidProfile profile;
    EXPECT_FALSE(profile.begin(limits(0, 60)));
}

} // namespace
} // namespace navigatr
