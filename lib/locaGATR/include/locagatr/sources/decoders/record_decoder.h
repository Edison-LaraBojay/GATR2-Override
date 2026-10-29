// record_decoder.h
// One control board record type into the SensorMap.

#pragma once
#include <cstdint>

#include "brainlink/wire.h"
#include "locagatr/pipeline.h"

namespace locagatr
{

class RecordDecoder {
  public:
    virtual ~RecordDecoder() = default;
    virtual uint8_t sensorType() const = 0;
    virtual void    decode(const brainlink::Record& record, SensorMap& sensors) const = 0;
};

} // namespace locagatr
