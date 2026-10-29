// mecanum_drive_gtest.cpp
// The wheel voltages; the motors are PROS only.

#include "navigatr/drives/mecanum_drive.h"

#include <gtest/gtest.h>

namespace navigatr
{
namespace
{

gatr2::DriveMotors motors() {
    gatr2::DriveMotors m;
    m.track_width = 12;
    m.wheelbase   = 12;
    return m;
}

Feedforward feedforward(double ks = 0) {
    Feedforward f;
    f.kv = 100;
    f.ks = ks;
    return f;
}

TEST(MecanumVoltages, ForwardRunsEveryWheelForward) {
    const auto mv = mecanumVoltages({10, 0, 0}, motors(), feedforward());
    for (double m : mv) {
        EXPECT_DOUBLE_EQ(m, 1000);
    }
}

TEST(MecanumVoltages, StrafeLeft) {
    const auto mv = mecanumVoltages({0, 10, 0}, motors(), feedforward());
    EXPECT_DOUBLE_EQ(mv[0], -1000); // front left
    EXPECT_DOUBLE_EQ(mv[1], 1000);  // front right
    EXPECT_DOUBLE_EQ(mv[2], 1000);  // back left
    EXPECT_DOUBLE_EQ(mv[3], -1000); // back right
}

TEST(MecanumVoltages, TurnCounterclockwise) {
    // 1 rad/s at 12 in from the center (half of track plus wheelbase).
    const auto mv = mecanumVoltages({0, 0, 180 / kPi}, motors(), feedforward());
    EXPECT_NEAR(mv[0], -1200, 1e-9);
    EXPECT_NEAR(mv[1], 1200, 1e-9);
    EXPECT_NEAR(mv[2], -1200, 1e-9);
    EXPECT_NEAR(mv[3], 1200, 1e-9);
}

TEST(MecanumVoltages, ScaledDownTogetherPastTheLimit) {
    const auto mv = mecanumVoltages({200, 100, 0}, motors(), feedforward()); // 30000 and 10000 mV asked
    EXPECT_DOUBLE_EQ(mv[1], 12000);
    EXPECT_DOUBLE_EQ(mv[0], 4000); // the same 1:3 ratio
}

TEST(MecanumVoltages, StaticFrictionOnlyWhenMoving) {
    EXPECT_DOUBLE_EQ(mecanumVoltages({0, 0, 0}, motors(), feedforward(500))[0], 0);
    EXPECT_DOUBLE_EQ(mecanumVoltages({-1, 0, 0}, motors(), feedforward(500))[0], -600);
}

TEST(WheelKv, FromTheMotorsUnlessSet) {
    gatr2::DriveMotors m = motors();
    m.motor_rpm          = 200;
    m.gear_ratio         = 1.0;
    m.wheel_diameter     = 4.0;
    // Top speed 200 / 60 * 4 pi = 41.9 in/s.
    EXPECT_NEAR(wheelKv(m, Feedforward{}), 12000 / (200.0 / 60 * 4 * kPi), 1e-9);
    EXPECT_DOUBLE_EQ(wheelKv(m, feedforward()), 100);
}

} // namespace
} // namespace navigatr
