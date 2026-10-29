// processors_gtest.cpp
// Each processor alone: raw readings in, inches and degrees out.

#include <gtest/gtest.h>

#include <cmath>

#include "locagatr/processors/drive_motor_processor.h"
#include "locagatr/processors/imu_processor.h"
#include "locagatr/processors/tracking_wheel_processor.h"
#include "locagatr/processors/wheel_pair_processor.h"

namespace locagatr
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

// 2 in wheels, 4000 counts per turn.
constexpr double kInchesPerCount = 2.0 * kPi / 4000.0;

TrackingWheel wheel(const char* name, uint8_t port, Axis axis, double offset) {
    TrackingWheel w;
    w.name   = name;
    w.port   = port;
    w.axis   = axis;
    w.offset = offset;
    return w;
}

Config wheels() {
    Config c;
    c.robot.tracking_wheels = {
        wheel("left", 0, Axis::kForward, 3.0),
        wheel("right", 2, Axis::kForward, -3.0),
        wheel("back", 1, Axis::kSideways, -2.0),
    };
    c.robot.wheel_pair = {"left", "right"};
    return c;
}

// --- Tracking wheels ---

TEST(TrackingWheelProcessor, CountsToInchesByAxis) {
    TrackingWheelProcessor p;
    ASSERT_TRUE(p.begin(wheels()));
    SensorMap s;
    s.encoder_counts[0] = 4000;
    s.encoder_counts[1] = -2000;
    MotionMap m;
    p.process(s, m);
    ASSERT_EQ(m.count, 2u); // right has no count
    EXPECT_NEAR(*m.find("left")->forward, 2 * kPi, 1e-9);
    EXPECT_DOUBLE_EQ(m.find("left")->offset, 3.0);
    EXPECT_FALSE(m.find("back")->forward.has_value());
    EXPECT_NEAR(*m.find("back")->lateral, -kPi, 1e-9);
    EXPECT_EQ(m.find("right"), nullptr);
}

TEST(TrackingWheelProcessor, ReversedWheel) {
    Config c                             = wheels();
    c.robot.tracking_wheels[0].direction = -1;
    TrackingWheelProcessor p;
    ASSERT_TRUE(p.begin(c));
    SensorMap s;
    s.encoder_counts[0] = 4000;
    MotionMap m;
    p.process(s, m);
    EXPECT_NEAR(*m.find("left")->forward, -2 * kPi, 1e-9);
}

TEST(TrackingWheelProcessor, TotalCarriesOnAcrossABoardRestart) {
    TrackingWheelProcessor p;
    ASSERT_TRUE(p.begin(wheels()));
    SensorMap s;
    s.encoder_counts[0] = 1000;
    MotionMap m1;
    p.process(s, m1);

    s.encoder_counts[0] = 0; // the board restarted
    s.board.restarted   = true;
    MotionMap m2;
    p.process(s, m2);
    EXPECT_NEAR(*m2.find("left")->forward, 1000 * kInchesPerCount, 1e-9);

    s.encoder_counts[0] = 500;
    s.board.restarted   = false;
    MotionMap m3;
    p.process(s, m3);
    EXPECT_NEAR(*m3.find("left")->forward, 1500 * kInchesPerCount, 1e-9);
}

TEST(TrackingWheelProcessor, NeedsNamedWheelsOnBoardPorts) {
    TrackingWheelProcessor p;
    EXPECT_FALSE(p.begin(Config{}));
    Config c                        = wheels();
    c.robot.tracking_wheels[1].name = "";
    EXPECT_FALSE(p.begin(c));
}

// --- Wheel pair ---

TEST(WheelPairProcessor, YawFromTheDifference) {
    TrackingWheelProcessor wheels_p;
    WheelPairProcessor     pair;
    ASSERT_TRUE(wheels_p.begin(wheels()));
    ASSERT_TRUE(pair.begin(wheels()));

    // 90 deg counterclockwise in place: left (3 in left) rolls back 3 * pi/2,
    // right forward as much.
    const double arc = 3.0 * kPi / 2.0;
    SensorMap    s;
    s.encoder_counts[0] = static_cast<int32_t>(std::lround(-arc / kInchesPerCount));
    s.encoder_counts[2] = static_cast<int32_t>(std::lround(arc / kInchesPerCount));
    MotionMap m;
    wheels_p.process(s, m);
    pair.process(s, m);
    ASSERT_NE(m.find("wheel pair"), nullptr);
    EXPECT_NEAR(*m.find("wheel pair")->yaw, 90, 0.02);
}

TEST(WheelPairProcessor, NothingWithoutBothWheels) {
    WheelPairProcessor pair;
    ASSERT_TRUE(pair.begin(wheels()));
    MotionMap m;
    m.add("left")->forward = 1.0;
    pair.process(SensorMap{}, m);
    EXPECT_EQ(m.find("wheel pair"), nullptr);
}

TEST(WheelPairProcessor, NeedsTwoForwardWheelsApart) {
    WheelPairProcessor pair;
    Config             c = wheels();
    c.robot.wheel_pair   = {"left", "back"}; // back is sideways
    EXPECT_FALSE(pair.begin(c));
    c.robot.wheel_pair = {"left", "missing"};
    EXPECT_FALSE(pair.begin(c));
}

// --- IMU ---

TEST(ImuProcessor, PassesYawPitchRoll) {
    ImuProcessor p;
    SensorMap    s;
    MotionMap    none;
    p.process(s, none);
    EXPECT_EQ(none.count, 0u);

    ImuSample imu;
    imu.rotation = 370;
    imu.pitch    = 2;
    imu.roll     = -1;
    s.imu        = imu;
    MotionMap m;
    p.process(s, m);
    EXPECT_DOUBLE_EQ(*m.find("IMU")->yaw, 370);
    EXPECT_DOUBLE_EQ(*m.find("IMU")->pitch, 2);
    EXPECT_DOUBLE_EQ(*m.find("IMU")->roll, -1);
}

// --- Drive motors ---

// 4 in wheels, direct drive: 4 pi in per 360 motor degrees.
constexpr double kDegreesPerInch = 360.0 / (4.0 * kPi);

// The mecanum drive in 2026-27: two motors per corner, some reversed.
Config mecanum() {
    Config c;
    c.robot.drive.front_left  = {-1, 2};
    c.robot.drive.front_right = {5, -6};
    c.robot.drive.back_left   = {3, -4};
    c.robot.drive.back_right  = {-7, 8};
    c.robot.drive.mecanum     = true;
    return c;
}

// Wheel travel per corner, in, into motor degrees as the encoders report them.
SensorMap drive(double fl, double fr, double bl, double br) {
    SensorMap  s;
    const auto put = [&](int8_t port, double inches) {
        const double degrees                      = inches * kDegreesPerInch;
        s.motor_positions[std::abs(port)] = port < 0 ? -degrees : degrees;
    };
    put(-1, fl), put(2, fl), put(5, fr), put(-6, fr);
    put(3, bl), put(-4, bl), put(-7, br), put(8, br);
    return s;
}

TEST(DriveMotorProcessor, Forward) {
    DriveMotorProcessor p;
    ASSERT_TRUE(p.begin(mecanum()));
    MotionMap m;
    p.process(drive(12, 12, 12, 12), m);
    EXPECT_NEAR(*m.find("drive")->forward, 12, 1e-9);
    EXPECT_NEAR(*m.find("drive")->lateral, 0, 1e-9);
}

TEST(DriveMotorProcessor, MecanumStrafeLeft) {
    DriveMotorProcessor p;
    ASSERT_TRUE(p.begin(mecanum()));
    MotionMap m;
    p.process(drive(-5, 5, 5, -5), m);
    EXPECT_NEAR(*m.find("drive")->forward, 0, 1e-9);
    EXPECT_NEAR(*m.find("drive")->lateral, 5, 1e-9);
}

TEST(DriveMotorProcessor, TurnInPlaceCancels) {
    DriveMotorProcessor p;
    ASSERT_TRUE(p.begin(mecanum()));
    MotionMap m;
    p.process(drive(-8, 8, -8, 8), m);
    EXPECT_NEAR(*m.find("drive")->forward, 0, 1e-9);
    EXPECT_NEAR(*m.find("drive")->lateral, 0, 1e-9);
}

TEST(DriveMotorProcessor, TankWithGearRatio) {
    Config c;
    c.robot.drive.front_left  = {1, 2};
    c.robot.drive.front_right = {-3, -4};
    c.robot.drive.gear_ratio  = 0.5; // wheel turns per motor turn
    DriveMotorProcessor p;
    ASSERT_TRUE(p.begin(c));
    SensorMap s;
    s.motor_positions[1] = s.motor_positions[2] = 360;
    s.motor_positions[3] = s.motor_positions[4] = -360; // reversed motors
    MotionMap m;
    p.process(s, m);
    EXPECT_NEAR(*m.find("drive")->forward, 0.5 * 4.0 * kPi, 1e-9);
    EXPECT_FALSE(m.find("drive")->lateral.has_value());
}

TEST(DriveMotorProcessor, MissingMotorGivesNothing) {
    DriveMotorProcessor p;
    ASSERT_TRUE(p.begin(mecanum()));
    SensorMap s = drive(1, 1, 1, 1);
    s.motor_positions[8].reset();
    MotionMap m;
    p.process(s, m);
    EXPECT_EQ(m.find("drive"), nullptr);
}

TEST(DriveMotorProcessor, NeedsBothSidesAndAllCornersForMecanum) {
    DriveMotorProcessor p;
    Config              one_side;
    one_side.robot.drive.front_left = {1};
    EXPECT_FALSE(p.begin(one_side));
    Config missing_corner = mecanum();
    missing_corner.robot.drive.back_right.clear();
    EXPECT_FALSE(p.begin(missing_corner));
}

} // namespace
} // namespace locagatr
