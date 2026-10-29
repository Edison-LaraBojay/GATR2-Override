// odometry_estimator_gtest.cpp
// Measurements in, pose out. The robot below writes the totals a set of
// processors would: left and right forward wheels 3 in either side of
// center, a sideways wheel 2 in ahead, the IMU's yaw.

#include "locagatr/estimators/odometry_estimator.h"

#include <gtest/gtest.h>

#include <cmath>

namespace locagatr
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

struct Robot {
    double left = 0, right = 0, back = 0; // in rolled
    double yaw   = 0;                      // deg
    bool   imu   = true;
    bool   has_back = true;

    MotionMap motion() const {
        MotionMap m;
        Measurement* l = m.add("left");
        l->forward     = left;
        l->offset      = 3.0;
        Measurement* r = m.add("right");
        r->forward     = right;
        r->offset      = -3.0;
        if (has_back) {
            Measurement* b = m.add("back");
            b->lateral     = back;
            b->offset      = 2.0;
        }
        if (imu) {
            Measurement* i = m.add("IMU");
            i->yaw         = yaw;
            i->pitch       = 1.5;
            i->roll        = -0.5;
        }
        return m;
    }

    // Robot frame motion with the turn, as the wheels see it.
    void move(double forward, double lateral, double turn_deg) {
        const double turn = turn_deg * kPi / 180.0;
        left += forward - 3.0 * turn;
        right += forward + 3.0 * turn;
        back += lateral + 2.0 * turn;
        yaw += turn_deg;
    }
};

Config chosen(std::vector<const char*> forward) {
    Config c;
    c.odometry.forward  = std::move(forward);
    c.odometry.lateral  = "back";
    c.odometry.yaw      = "IMU";
    c.odometry.attitude = "IMU";
    return c;
}

struct Started {
    OdometryEstimator estimator;
    explicit Started(std::vector<const char*> forward = {"left"}) {
        EXPECT_TRUE(estimator.begin(chosen(std::move(forward))));
    }
};

TEST(OdometryEstimator, NeedsSomethingChosen) {
    OdometryEstimator e;
    EXPECT_FALSE(e.begin(Config{}));
}

TEST(OdometryEstimator, FirstTickSetsBaselinesWithoutMoving) {
    Started s;
    Robot   robot;
    robot.left        = 500;
    robot.yaw         = 37;
    const Estimate e  = s.estimator.update(robot.motion(), SensorMap{});
    EXPECT_TRUE(e.tracking);
    EXPECT_DOUBLE_EQ(e.pose.x, 0);
    EXPECT_DOUBLE_EQ(e.pose.heading, 0); // the first yaw keeps the set heading
    EXPECT_DOUBLE_EQ(e.pose.pitch, 1.5);
    EXPECT_DOUBLE_EQ(e.pose.roll, -0.5);
}

TEST(OdometryEstimator, DrivesStraight) {
    Started s;
    Robot   robot;
    s.estimator.update(robot.motion(), SensorMap{});
    robot.move(10, 0, 0);
    const Estimate e = s.estimator.update(robot.motion(), SensorMap{});
    EXPECT_NEAR(e.pose.x, 10, 1e-9);
    EXPECT_NEAR(e.motion.forward, 10, 1e-9);
}

TEST(OdometryEstimator, TurnInPlaceStaysPut) {
    Started s;
    Robot   robot;
    s.estimator.update(robot.motion(), SensorMap{});
    for (int i = 0; i < 9; ++i) {
        robot.move(0, 0, 10);
        s.estimator.update(robot.motion(), SensorMap{});
    }
    const Pose p = s.estimator.update(robot.motion(), SensorMap{}).pose;
    EXPECT_NEAR(p.x, 0, 1e-9);
    EXPECT_NEAR(p.y, 0, 1e-9);
    EXPECT_NEAR(p.heading, 90, 1e-9);
}

TEST(OdometryEstimator, TwoForwardWheelsAveraged) {
    Started s({"left", "right"});
    Robot   robot;
    s.estimator.update(robot.motion(), SensorMap{});
    robot.left += 4;
    robot.right += 2;
    EXPECT_NEAR(s.estimator.update(robot.motion(), SensorMap{}).pose.x, 3, 1e-9);
}

TEST(OdometryEstimator, QuarterCircleEndsWhereTheArcDoes) {
    Started      s;
    Robot        robot;
    const double radius = 24;
    s.estimator.update(robot.motion(), SensorMap{});
    for (int i = 0; i < 30; ++i) {
        robot.move(radius * 3.0 * kPi / 180.0, 0, 3.0);
        s.estimator.update(robot.motion(), SensorMap{});
    }
    const Pose p = s.estimator.update(robot.motion(), SensorMap{}).pose;
    EXPECT_NEAR(p.x, radius, 1e-9);
    EXPECT_NEAR(p.y, radius, 1e-9);
    EXPECT_NEAR(p.heading, 90, 1e-9);
}

TEST(OdometryEstimator, StrafesLeftWhileFacingUp) {
    Started s;
    Robot   robot;
    s.estimator.update(robot.motion(), SensorMap{});
    robot.move(0, 0, 90);
    s.estimator.update(robot.motion(), SensorMap{});
    robot.move(0, 10, 0);
    const Pose p = s.estimator.update(robot.motion(), SensorMap{}).pose;
    EXPECT_NEAR(p.x, -10, 1e-9); // facing +y, left is -x
    EXPECT_NEAR(p.y, 0, 1e-9);
}

TEST(OdometryEstimator, MissingMeasurementWaitsThenTakesTheWholeDifference) {
    Started s;
    Robot   robot;
    s.estimator.update(robot.motion(), SensorMap{});
    robot.has_back = false;
    robot.move(5, 0, 20);
    const Estimate waiting = s.estimator.update(robot.motion(), SensorMap{});
    EXPECT_FALSE(waiting.tracking);
    EXPECT_STREQ(waiting.missing, "back"); // says which one
    EXPECT_DOUBLE_EQ(waiting.pose.x, 0);
    EXPECT_NEAR(waiting.pose.heading, 20, 1e-9); // heading still follows the IMU
    robot.has_back = true;
    robot.move(5, 0, 0);
    const Pose p = s.estimator.update(robot.motion(), SensorMap{}).pose;
    // 5 in along a 20 deg arc, then 5 in straight at 20 deg, all in one step:
    // the arc step treats it as one 10 in arc of 20 deg.
    EXPECT_NEAR(std::hypot(p.x, p.y), 10 * std::sin(kPi / 18) / (kPi / 18), 1e-9);
}

TEST(OdometryEstimator, SetPoseContinuesFromThere) {
    Started s;
    Robot   robot;
    robot.yaw = 37;
    s.estimator.update(robot.motion(), SensorMap{});
    Pose start;
    start.x       = 12;
    start.y       = 30;
    start.heading = 180;
    s.estimator.setPose(start);
    robot.move(6, 0, 0);
    const Pose p = s.estimator.update(robot.motion(), SensorMap{}).pose;
    EXPECT_NEAR(p.x, 6, 1e-9);
    EXPECT_NEAR(p.y, 30, 1e-9);
    EXPECT_NEAR(p.heading, 180, 1e-9);
    robot.move(0, 0, -270);
    EXPECT_NEAR(s.estimator.update(robot.motion(), SensorMap{}).pose.heading, 270, 1e-9);
}

TEST(OdometryEstimator, HeadingHoldsWithoutTheYawMeasurement) {
    Started s;
    Robot   robot;
    s.estimator.update(robot.motion(), SensorMap{});
    robot.move(0, 0, 45);
    s.estimator.update(robot.motion(), SensorMap{});
    robot.imu = false;
    robot.move(10, 0, 0);
    const Estimate e = s.estimator.update(robot.motion(), SensorMap{});
    const Pose     p = e.pose;
    EXPECT_STREQ(e.missing, "IMU");
    EXPECT_NEAR(p.heading, 45, 1e-9);
    EXPECT_NEAR(p.x, 10 * std::cos(kPi / 4), 1e-9);
}

} // namespace
} // namespace locagatr
