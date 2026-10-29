// control_board_source.h
// The control board: one GET_READINGS per tick, each record to the decoder for
// its sensor type. Records with no decoder are counted in SensorMap::board.

#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "locagatr/link.h"
#include "locagatr/pipeline.h"
#include "locagatr/sources/decoders/encoder_count_decoder.h"
#include "locagatr/sources/decoders/record_decoder.h"

namespace locagatr
{

class ControlBoardSource : public Source {
  public:
    ControlBoardSource() : decoders_{&encoder_counts_} {}
    explicit ControlBoardSource(std::vector<const RecordDecoder*> decoders)
        : decoders_(std::move(decoders)) {}
    ControlBoardSource(const ControlBoardSource&)            = delete;
    ControlBoardSource& operator=(const ControlBoardSource&) = delete;

    const char* name() const override { return "control board"; }
    bool        begin(const Config& config) override;
    void        read(SensorMap& sensors) override;

    // Part of read(), for tests.
    void decode(const Poll& poll, const LinkStats& stats, SensorMap& sensors);

    // Part of begin(), for tests.
    bool buildTable();

  private:
    EncoderCountDecoder                   encoder_counts_;
    std::vector<const RecordDecoder*>     decoders_;
    std::array<const RecordDecoder*, 256> by_type_{};
    std::optional<Link>                   link_;
    uint32_t                              unhandled_ = 0;
    uint8_t                               last_type_ = 0;
    uint8_t                               last_port_ = 0;
};

} // namespace locagatr
