// pipeline.h
// The localization loop. The Localizer task calls tick() rate_hz times a
// second (pros/localizer.cpp). Portable.
//
//   Tick Pipeline::tick(now_us) {
//       for (Source* s : sources_)       s->read(t.sensors);              // 1. acquisition
//       for (Processor* p : processors_) p->process(t.sensors, t.motion); // 2. processing
//       t.estimate = estimator_->update(t.motion, t.sensors);             // 3. estimation
//   }
//
// Stages: sources/source.h, processors/processor.h, estimators/estimator.h.
// What runs in each: Config.

#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "locagatr/config.h"
#include "locagatr/estimators/estimator.h"
#include "locagatr/processors/processor.h"
#include "locagatr/sources/source.h"

namespace locagatr
{

constexpr std::size_t kMaxSources    = 8;
constexpr std::size_t kMaxProcessors = 8;

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
