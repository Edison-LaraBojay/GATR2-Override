// pipeline.h
// The path following pipeline, per move in a Route:
//
//   1. path      segments -> Path
//   2. profile   Path -> Trajectory          (Profile)
//   then every tick:
//   3. follow    pose + target -> command    (Follower)
//   4. drive     command -> motors           (Drive)
//
// Portable: pose and time come in through Io.

#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "navigatr/config.h"
#include "navigatr/route.h"
#include "navigatr/types.h"

namespace navigatr
{

// --- 1. Path ---

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

// --- 2. Profile ---

struct TrajectoryPoint {
    double t            = 0; // s from the start of the move
    double s            = 0; // in along the path
    double x            = 0;
    double y            = 0;
    double v            = 0; // in/s along the path
    double tangent      = 0; // deg
    double heading      = 0; // deg, continuous
    double heading_rate = 0; // deg/s
};

struct Trajectory {
    std::vector<TrajectoryPoint> points;

    double duration() const { return points.empty() ? 0.0 : points.back().t; }
    // Past the end: the end, at rest.
    TrajectoryPoint at(double t) const;
};

class Profile {
  public:
    virtual ~Profile() = default;
    virtual const char* name() const = 0;
    virtual bool        begin(const Config&) { return true; }
    virtual Trajectory  build(const Path& path) const = 0;
};

// --- 3. Follow ---

// Where the robot should be now, and how fast it should be moving.
struct Target {
    double x         = 0;
    double y         = 0;
    double heading   = 0; // deg
    double vx        = 0; // in/s, field frame
    double vy        = 0; // in/s
    double turn_rate = 0; // deg/s
};

class Follower {
  public:
    virtual ~Follower() = default;
    virtual const char*   name() const = 0;
    virtual bool          begin(const Config&) { return true; }
    virtual void          reset() {} // a new step starts
    virtual ChassisSpeeds update(const Pose& pose, const Target& target, double dt) = 0;
};

// --- 4. Drive ---

class Drive {
  public:
    virtual ~Drive() = default;
    virtual const char* name() const = 0;
    virtual bool        begin(const Config&) { return true; }
    virtual void        apply(const ChassisSpeeds& speeds) = 0;
    virtual void        stop() = 0;
};

// --- The loop ---

struct Io {
    std::function<Pose()>         pose;        // the localizer's newest pose
    std::function<uint32_t()>     now_ms;
    std::function<void(uint32_t)> sleep_until; // ms, as now_ms counts
    std::function<bool()>         cancelled;   // optional. True: stop the route
    std::function<void(double)>   progress;    // optional. Driving distance covered, 0 to 1
};

// Stages 1 and 2 for the whole route, before it moves.
struct MovePlan {
    Path       path;
    Trajectory trajectory;
};

struct Plan {
    std::vector<MovePlan> moves;        // one per move step, in route order
    double                length   = 0; // in, every move
    double                duration = 0; // s, every move as profiled; turns and waits not counted
};

// How a run ended.
struct Outcome {
    bool     cancelled       = false;
    uint32_t timed_out       = 0;  // steps that gave up
    int      first_timed_out = -1; // index into Route::steps()

    bool ok() const { return !cancelled && timed_out == 0; }
};

class Pipeline {
  public:
    Pipeline(const Config& config, Io io);
    ~Pipeline();
    Pipeline(const Pipeline&)            = delete;
    Pipeline& operator=(const Pipeline&) = delete;

    // False if a stage failed to begin or there is no drive.
    bool begin();

    // After begin().
    Plan plan(const Route& route) const;

    // Every step in order, in the calling task. A timed out step does not stop
    // the route; a cancel does.
    Outcome run(const Route& route);

  private:
    enum class End { kSettled, kTimedOut, kCancelled };

    End  move(const MovePlan& plan, double done, double total);
    End  turn(const Step& step);
    End  wait(const Step& step);
    bool cancelled() const { return io_.cancelled && io_.cancelled(); }
    End track(const std::function<Target(double t, bool& arrived)>& target, double timeout_s,
              const std::function<void(double t)>& on_tick);

    Config                    config_;
    Io                        io_;
    std::unique_ptr<Profile>  own_profile_;
    std::unique_ptr<Follower> own_follower_;
    Profile*                  profile_  = nullptr;
    Follower*                 follower_ = nullptr;
    Drive*                    drive_    = nullptr;
};

} // namespace navigatr
