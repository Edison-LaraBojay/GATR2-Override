// pipeline.h
// The localization pipeline, one tick of Pipeline::tick():
//
//   1. acquisition  Source[]     -> SensorMap  raw readings
//   2. processing   Processor[]  -> MotionMap  inches and degrees
//   3. estimation   Estimator    -> Estimate   the pose
//
// Portable.

#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <vector>

#include "locagatr/config.h"
#include "locagatr/types.h"

namespace locagatr
{

// Control board connectors a reading can come from (0 = TW0).
constexpr std::size_t kBoardPorts = 8;
// Smart Ports, indexed 1..21.
constexpr std::size_t kSmartPorts = 22;

constexpr std::size_t kMaxSources      = 8;
constexpr std::size_t kMaxProcessors   = 8;
constexpr std::size_t kMaxMeasurements = 12;

// --- 1. Acquisition ---

struct ImuSample {
    double  rotation = 0; // deg counterclockwise since calibration, unwrapped
    double  pitch    = 0; // deg
    double  roll     = 0; // deg
    Vector3 gyro;         // deg/s, sensor axes as PROS reports them
    Vector3 accel;        // g, sensor axes as PROS reports them
};

struct BoardStatus {
    bool     fresh               = false; // the board is answering
    bool     restarted           = false; // its counts started over this tick
    uint32_t requests            = 0;
    uint32_t replies             = 0;
    uint32_t timeouts            = 0;
    uint32_t unhandled_records   = 0; // records with no decoder, since start
    uint8_t  last_unhandled_type = 0;
    uint8_t  last_unhandled_port = 0;
};

// Every raw reading this tick, each source filling its own fields.
struct SensorMap {
    uint64_t    time_us = 0;
    double      dt      = 0; // s since the previous tick; 0 on the first
    BoardStatus board;
    std::array<std::optional<int32_t>, kBoardPorts> encoder_counts{};  // by board connector
    std::array<std::optional<double>, kSmartPorts>  motor_positions{}; // deg of the motor output, by port
    std::optional<ImuSample>                         imu;
};

class Source {
  public:
    virtual ~Source() = default;
    virtual const char* name() const = 0;
    // From Localizer::start(). False: left out.
    virtual bool begin(const Config&) { return true; }
    virtual void read(SensorMap& sensors) = 0;
};

// --- 2. Processing ---

// One measurement, as totals since start. Unset: not measured this tick.
struct Measurement {
    const char*           name = "";
    std::optional<double> forward; // in rolled forward
    std::optional<double> lateral; // in rolled left
    std::optional<double> yaw;     // deg counterclockwise, unwrapped
    std::optional<double> pitch;   // deg
    std::optional<double> roll;    // deg
    // in. Wheel position: forward wheels left of center, sideways ahead of it.
    double offset = 0;
};

struct MotionMap {
    std::array<Measurement, kMaxMeasurements> entries{};
    std::size_t                               count = 0;

    // A new, empty measurement; null when full.
    Measurement* add(const char* name) {
        if (count == entries.size()) {
            return nullptr;
        }
        Measurement& m = entries[count++];
        m              = Measurement{};
        m.name         = name;
        return &m;
    }

    const Measurement* find(const char* name) const {
        for (std::size_t i = 0; name != nullptr && i < count; ++i) {
            if (std::strcmp(entries[i].name, name) == 0) {
                return &entries[i];
            }
        }
        return nullptr;
    }
};

class Processor {
  public:
    virtual ~Processor() = default;
    virtual const char* name() const = 0;
    // False: left out.
    virtual bool begin(const Config&) { return true; }
    // Earlier processors have already added theirs.
    virtual void process(const SensorMap& sensors, MotionMap& motion) = 0;
};

// --- 3. Estimation ---

struct Estimate {
    Pose       pose;
    BodyMotion motion;           // this tick
    bool       tracking = false; // had every measurement it needs this tick
    // The chosen measurement it did not get this tick, by name.
    const char* missing = nullptr;
};

class Estimator {
  public:
    virtual ~Estimator() = default;
    virtual const char* name() const = 0;
    virtual bool        begin(const Config&) { return true; }
    virtual Estimate    update(const MotionMap& motion, const SensorMap& sensors) = 0;
    virtual void setPose(const Pose& pose) = 0;
};

// --- The loop ---

struct Tick {
    SensorMap sensors;
    MotionMap motion;
    Estimate  estimate;
};

class Pipeline {
  public:
    explicit Pipeline(const Config& config);
    ~Pipeline();
    Pipeline(const Pipeline&)            = delete;
    Pipeline& operator=(const Pipeline&) = delete;

    // False if any implementation failed to begin; the rest still run.
    bool begin();

    Tick tick(uint64_t now_us);

    void setPose(const Pose& pose);

    // Bit i: entry i began and runs.
    uint32_t sourcesRunning() const { return sources_running_; }
    uint32_t processorsRunning() const { return processors_running_; }
    bool     estimatorRunning() const { return estimator_running_; }

  private:
    Config                     config_;
    std::vector<Source*>       sources_;
    std::vector<Processor*>    processors_;
    std::unique_ptr<Estimator> own_estimator_;
    Estimator*                 estimator_          = nullptr;
    uint32_t                   sources_running_    = 0;
    uint32_t                   processors_running_ = 0;
    bool                       estimator_running_  = false;
    std::optional<uint64_t>    previous_us_;
};

} // namespace locagatr
