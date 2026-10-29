// viewer.h
// spectaGATR for a PROS program: pose, footprint and path to the laptop
// viewer, from one background task. Each line is one write() to stdout, one
// COBS packet. Never blocks a caller. Source in pros/.
//
//   spectagatr::Viewer viz(vizConfig());    // global
//   void initialize() { viz.start(); }
//   viz.updatePose(x, y, heading);          // any task
//   viz.setPath(points);
//
// Pose lines first, only while the stdio FIFO is nearly empty, so a slow
// reader gets fresh poses. Robot and path lines only while it is empty.

#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <vector>

#include "spectagatr/publisher.h"

namespace spectagatr
{

struct ViewerConfig {
    Hertz    rate_hz = kDefaultRateHz; // pose lines per second, 1..50
    RobotBox robot;                    // drawn around the pose point; all zero keeps the viewer default
};

class Viewer {
  public:
    Viewer();
    explicit Viewer(const ViewerConfig& config);

    Viewer(const Viewer&)            = delete;
    Viewer& operator=(const Viewer&) = delete;

    // From initialize() or later, not from a global constructor. The first call
    // picks the session id and creates the task (priority TASK_PRIORITY_DEFAULT
    // - 1); later calls resume output. False when the task could not be created.
    bool start();
    // Stops output; the task stays parked until the next start().
    void stop();
    bool running() const;

    // Stamped with pros::millis(). Any task, never blocks.
    void updatePose(Meters x, Meters y, Radians heading);
    void updatePose(Meters x, Meters y, const Attitude& attitude);
    // After re-placing the robot or resetting odometry: the viewer starts a new trail.
    void resetTrail();

    // The planned path, field meters, in order; the viewer joins the points
    // with straight lines. At most kMaxPathPoints are sent. Any task; one at
    // a time.
    void setPath(const std::vector<Point>& points);
    void setPath(const std::vector<std::array<Meters, 2>>& points); // {x, y}
    void setPath(const Point* points, std::size_t count);
    void clearPath();

    // A new footprint while running.
    void setRobot(const RobotBox& box);

    void           setRate(Hertz rate); // 1..50 Hz, default 10
    Hertz          rate() const;
    PublisherStats stats() const;
    SessionId      session() const; // 0 before the first start()

  private:
    static void run(void* self);

    Publisher         publisher_;
    std::atomic<bool> enabled_{false};
    std::atomic<bool> task_created_{false};
};

} // namespace spectagatr
