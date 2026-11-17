// pipeline.cpp
// The localization loop.

#include "locagatr/pipeline.h"

#include "locagatr/estimators/odometry_estimator.h"

namespace locagatr
{

Tick Pipeline::tick(uint64_t now_us) {
    Tick t;
    t.sensors.time_us = now_us;
    t.sensors.dt      = previous_us_ ? (now_us - *previous_us_) / 1e6 : 0.0;
    previous_us_      = now_us;

    // 1. Acquisition.
    for (Source* source : sources_) {
        source->read(t.sensors);
    }
    // 2. Processing.
    for (Processor* processor : processors_) {
        processor->process(t.sensors, t.motion);
    }
    // 3. Estimation.
    t.estimate = estimator_->update(t.motion, t.sensors);
    return t;
}

namespace
{

// Begins the enabled entries of one list. Bit i of running: entry i runs.
template <typename T>
bool beginAll(const std::vector<Enabled<T>>& list, std::size_t max, const Config& config,
              std::vector<T*>& out, uint32_t& running) {
    bool ok = list.size() <= max;
    out.clear();
    running = 0;
    for (std::size_t i = 0; i < list.size() && i < max; ++i) {
        const Enabled<T>& entry = list[i];
        if (!entry.enabled || entry.implementation == nullptr) {
            continue;
        }
        if (entry.implementation->begin(config)) {
            out.push_back(entry.implementation);
            running |= 1u << i;
        } else {
            ok = false;
        }
    }
    return ok;
}

} // namespace

Pipeline::Pipeline(const Config& config) : config_(config) {}

Pipeline::~Pipeline() = default;

bool Pipeline::begin() {
    bool ok = beginAll(config_.sources, kMaxSources, config_, sources_, sources_running_);
    ok = beginAll(config_.processors, kMaxProcessors, config_, processors_, processors_running_) &&
         ok;

    estimator_ = config_.estimator;
    if (estimator_ == nullptr) {
        own_estimator_ = std::make_unique<OdometryEstimator>();
        estimator_     = own_estimator_.get();
    }
    estimator_running_ = estimator_->begin(config_);
    ok                 = estimator_running_ && ok;

    setPose(config_.start_pose);
    return ok;
}

void Pipeline::setPose(const Pose& pose) {
    if (estimator_ != nullptr) {
        estimator_->setPose(pose);
    }
}

} // namespace locagatr
