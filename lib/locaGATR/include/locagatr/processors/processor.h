// processor.h
// Stage 2, processing: Processor[] -> MotionMap, inches and degrees. Portable.

#pragma once
#include <array>
#include <cstddef>
#include <cstring>
#include <optional>

#include "locagatr/config.h"
#include "locagatr/sources/source.h"

namespace locagatr
{

constexpr std::size_t kMaxMeasurements = 12;

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

} // namespace locagatr
