// pipeline_gtest.cpp

#include "locagatr/pipeline.h"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

namespace locagatr
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

using Log = std::vector<std::string>;

struct FakeSource : Source {
    FakeSource(Log& log, std::string id, bool starts = true) : log(log), id(id), starts(starts) {}
    const char* name() const override { return id.c_str(); }
    bool        begin(const Config& config) override {
        seen_imu_port = config.robot.imu_port;
        return starts;
    }
    void read(SensorMap& m) override {
        log.push_back("read " + id);
        m.encoder_counts[0] = 42;
    }
    Log&        log;
    std::string id;
    bool        starts;
    uint8_t     seen_imu_port = 0;
};

struct FakeProcessor : Processor {
    FakeProcessor(Log& log, std::string id) : log(log), id(id) {}
    const char* name() const override { return id.c_str(); }
    void        process(const SensorMap& sensors, MotionMap& motion) override {
        log.push_back("process " + id);
        if (Measurement* m = motion.add(id.c_str())) {
            m->forward = sensors.encoder_counts[0].value_or(0);
        }
    }
    Log&        log;
    std::string id;
};

struct FakeEstimator : Estimator {
    explicit FakeEstimator(Log& log) : log(log) {}
    const char* name() const override { return "fake"; }
    Estimate    update(const MotionMap& motion, const SensorMap&) override {
        log.push_back("estimate");
        Estimate e;
        e.pose = set;
        if (const Measurement* m = motion.find("p1")) {
            e.pose.x = set.x + m->forward.value_or(0);
        }
        return e;
    }
    void setPose(const Pose& pose) override { set = pose; }
    Log& log;
    Pose set;
};

TEST(Pipeline, AcquisitionThenProcessingThenEstimation) {
    Log           log;
    FakeSource    a(log, "a");
    FakeSource    b(log, "b");
    FakeProcessor p1(log, "p1");
    FakeProcessor p2(log, "p2");
    FakeEstimator estimator(log);
    Config        c;
    c.sources        = {{&a}, {&b}};
    c.processors     = {{&p1}, {&p2}};
    c.estimator      = &estimator;
    c.robot.imu_port = 15;
    Pipeline pipeline(c);
    ASSERT_TRUE(pipeline.begin());
    EXPECT_EQ(a.seen_imu_port, 15); // implementations read their part of the config
    EXPECT_EQ(pipeline.sourcesRunning(), 0b11u);

    const Tick t = pipeline.tick(1000);
    EXPECT_EQ(log, (Log{"read a", "read b", "process p1", "process p2", "estimate"}));
    EXPECT_EQ(t.motion.count, 2u);
    EXPECT_DOUBLE_EQ(*t.motion.find("p2")->forward, 42);
    EXPECT_EQ(t.motion.find("p3"), nullptr);
    EXPECT_DOUBLE_EQ(t.estimate.pose.x, 42); // the estimator sees what processing made
}

TEST(Pipeline, DisabledImplementationsNeverRun) {
    Log           log;
    FakeSource    on(log, "on");
    FakeSource    off(log, "off");
    FakeProcessor p1(log, "p1");
    FakeProcessor p2(log, "p2");
    FakeEstimator estimator(log);
    Config        c;
    c.sources    = {{&off, false}, {&on, true}};
    c.processors = {{&p1, false}, {&p2, true}};
    c.estimator  = &estimator;
    Pipeline pipeline(c);
    EXPECT_TRUE(pipeline.begin()); // disabled is not a failure
    EXPECT_EQ(pipeline.sourcesRunning(), 0b10u);
    EXPECT_EQ(pipeline.processorsRunning(), 0b10u);
    pipeline.tick(0);
    EXPECT_EQ(log, (Log{"read on", "process p2", "estimate"}));
}

TEST(Pipeline, SourceThatFailsToBeginIsLeftOut) {
    Log           log;
    FakeSource    good(log, "good");
    FakeSource    bad(log, "bad", false);
    FakeEstimator estimator(log);
    Config        c;
    c.sources   = {{&bad}, {&good}};
    c.estimator = &estimator;
    Pipeline pipeline(c);
    EXPECT_FALSE(pipeline.begin());
    EXPECT_EQ(pipeline.sourcesRunning(), 0b10u);
    pipeline.tick(0);
    EXPECT_EQ(log, (Log{"read good", "estimate"}));
}

TEST(Pipeline, DefaultEstimatorWithNothingChosenFailsBegin) {
    Config c;
    c.start_pose.x = 5;
    Pipeline pipeline(c);
    EXPECT_FALSE(pipeline.begin()); // OdometryEstimator needs Config::odometry
    EXPECT_DOUBLE_EQ(pipeline.tick(0).estimate.pose.x, 5);
}

TEST(Pipeline, StartAndSetPoseReachTheEstimator) {
    Log           log;
    FakeEstimator estimator(log);
    Config        c;
    c.estimator    = &estimator;
    c.start_pose.y = 3;
    Pipeline pipeline(c);
    pipeline.begin();
    EXPECT_DOUBLE_EQ(estimator.set.y, 3);
    Pose p;
    p.y = 7;
    pipeline.setPose(p);
    EXPECT_DOUBLE_EQ(pipeline.tick(0).estimate.pose.y, 7);
}

TEST(Pipeline, DtIsTheTimeBetweenTicks) {
    Log           log;
    FakeEstimator estimator(log);
    Config        c;
    c.estimator = &estimator;
    Pipeline pipeline(c);
    pipeline.begin();
    EXPECT_DOUBLE_EQ(pipeline.tick(5000).sensors.dt, 0);
    EXPECT_DOUBLE_EQ(pipeline.tick(15000).sensors.dt, 0.010);
}

TEST(Pipeline, MotionMapStopsAddingWhenFull) {
    MotionMap motion;
    for (std::size_t i = 0; i < kMaxMeasurements; ++i) {
        ASSERT_NE(motion.add("m"), nullptr);
    }
    EXPECT_EQ(motion.add("one too many"), nullptr);
}

TEST(Pipeline, AdvanceFollowsTheArc) {
    Pose start;
    start.heading = 350;
    BodyMotion quarter;
    quarter.forward = 10.0 * kPi / 2.0; // radius 10
    quarter.turn    = 90;
    const Pose p = advance(start, quarter);
    EXPECT_NEAR(p.heading, 80, 1e-9);
    // From heading 350, a left quarter circle of radius 10: 10 forward, 10 left.
    const double h = -10.0 * kPi / 180.0;
    EXPECT_NEAR(p.x, 10 * std::cos(h) - 10 * std::sin(h), 1e-9);
    EXPECT_NEAR(p.y, 10 * std::sin(h) + 10 * std::cos(h), 1e-9);
}

TEST(Pipeline, AdvanceStraightKeepsHeading) {
    Pose start;
    start.heading = 90;
    BodyMotion straight;
    straight.forward = 5;
    straight.left    = 2;
    const Pose p     = advance(start, straight);
    EXPECT_NEAR(p.x, -2, 1e-9);
    EXPECT_NEAR(p.y, 5, 1e-9);
    EXPECT_NEAR(p.heading, 90, 1e-9);
}

} // namespace
} // namespace locagatr
