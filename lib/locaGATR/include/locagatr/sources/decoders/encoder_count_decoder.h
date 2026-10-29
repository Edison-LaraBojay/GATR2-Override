// encoder_count_decoder.h
// Encoder records to SensorMap::encoder_counts, by board connector.

#pragma once
#include "locagatr/sources/decoders/record_decoder.h"
#include "records/encoder.h"

namespace locagatr
{

class EncoderCountDecoder : public RecordDecoder {
  public:
    uint8_t sensorType() const override { return records::encoder::kSensorType; }

    void decode(const brainlink::Record& record, SensorMap& sensors) const override {
        if (record.port < kBoardPorts) {
            sensors.encoder_counts[record.port] =
                records::encoder::read(record.data.data(), record.size);
        }
    }
};

} // namespace locagatr
