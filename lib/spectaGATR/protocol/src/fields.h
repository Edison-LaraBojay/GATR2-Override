// fields.h
// Shared by the line codecs: the frame around every SG?1 line and its decimal
// and hex fields. Internal to protocol/.

#pragma once
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "spectagatr/protocol/pose_line.h"

namespace spectagatr::protocol::fields
{

// Exactly digits uppercase hex digits.
bool parseHex(std::string_view text, std::size_t digits, uint32_t& out);

// 1 to 10 decimal digits, value <= max.
bool parseUnsigned(std::string_view text, uint64_t max, uint64_t& out);

// Optional '-' then 1 to 10 digits, within int32.
bool parseSigned(std::string_view text, int32_t& out);

// Splits on single spaces; false unless exactly count fields, none empty.
bool split(std::string_view text, std::string_view* out, std::size_t count);

// Checks the marker for kind ('P', 'R', 'T'), the '*' and the CRC. On kNone,
// payload is the text between the marker and the '*'. A trailing '\r' is
// ignored.
DecodeError openFrame(std::string_view line, char kind, std::string_view& payload);

// Appends "*CRC\n" to body[0, length) into out; the whole line or 0 when it
// would not fit kMaxLineLength or cap.
std::size_t closeFrame(const char* body, std::size_t length, char* out, std::size_t cap);

} // namespace spectagatr::protocol::fields
