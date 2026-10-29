// aux_lines_gtest.cpp
// Robot footprint and path point lines: round trips, ranges, CRC, and the
// scanner handing out every line kind by its marker letter.

#include "spectagatr/protocol/path_line.h"
#include "spectagatr/protocol/robot_line.h"

#include <cstdio>
#include <gtest/gtest.h>
#include <string>
#include <vector>

using namespace spectagatr::protocol;

namespace
{

constexpr SessionId kSession = 0x5A3C19E2;

RobotRecord robot() {
    RobotRecord r;
    r.session   = kSession;
    r.front_mm  = 170;
    r.back_mm   = 230;
    r.left_mm   = 150;
    r.right_mm  = 150;
    r.height_mm = 250;
    return r;
}

PathPointRecord point(PathIndex index, PathIndex count, Millimeters x, Millimeters y) {
    PathPointRecord r;
    r.session = kSession;
    r.path_id = 7;
    r.index   = index;
    r.count   = count;
    r.x_mm    = x;
    r.y_mm    = y;
    return r;
}

std::string encode(const RobotRecord& r) {
    char              buf[kMaxLineLength + 8];
    const std::size_t n = encodeRobotLine(r, buf, sizeof(buf));
    return std::string(buf, n);
}

std::string encode(const PathPointRecord& r) {
    char              buf[kMaxLineLength + 8];
    const std::size_t n = encodePathLine(r, buf, sizeof(buf));
    return std::string(buf, n);
}

std::string body(const std::string& line) { return line.substr(0, line.size() - 1); }

std::vector<std::string> scanAll(LineScanner& scanner, const std::string& bytes) {
    std::vector<std::string> lines;
    scanner.feed(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size(),
                 [&](std::string_view l) { lines.emplace_back(l); });
    return lines;
}

TEST(RobotLine, RoundTripAndKnownText) {
    const std::string line = encode(robot());
    EXPECT_EQ(line.substr(0, 31), "SGR1 5A3C19E2 170 230 150 150 2");
    EXPECT_EQ(line.back(), '\n');
    EXPECT_EQ(line[line.size() - 6], '*');
    const RobotDecodeResult decoded = decodeRobotLine(body(line));
    EXPECT_EQ(decoded.error, DecodeError::kNone);
    EXPECT_TRUE(decoded.record == robot());
    EXPECT_EQ(lineKind(line), LineKind::kRobot);
    EXPECT_EQ(decodeLine(body(line)).error, DecodeError::kMalformed); // not a pose
}

TEST(RobotLine, RangesAndShape) {
    RobotRecord r = robot();
    r.front_mm    = 0; // a box flush with its pose point at the front is fine
    EXPECT_TRUE(valid(r));
    r.back_mm = 0;
    EXPECT_FALSE(valid(r)); // no length
    r         = robot();
    r.left_mm = r.right_mm = 0;
    EXPECT_FALSE(valid(r)); // no width
    r           = robot();
    r.height_mm = 0;
    EXPECT_FALSE(valid(r));
    r           = robot();
    r.height_mm = kMaxRobotMm + 1;
    EXPECT_FALSE(valid(r));
    EXPECT_EQ(encodeRobotLine(r, nullptr, 0), 0u);
    r.height_mm = kMaxRobotMm;
    EXPECT_TRUE(valid(r));
    r.session = 0;
    EXPECT_FALSE(valid(r));

    // On the wire the range is checked after the CRC.
    const std::string big = "SGR1 5A3C19E2 170 230 150 150 5001";
    char              crc[8];
    std::snprintf(crc, sizeof(crc), "*%04X", crc16(big));
    EXPECT_EQ(decodeRobotLine(big + crc).error, DecodeError::kOutOfRange);
    EXPECT_EQ(decodeRobotLine(big + "*0000").error, DecodeError::kBadCrc);
    const std::string few = "SGR1 5A3C19E2 170 230 150"; // a field short, CRC fine
    std::snprintf(crc, sizeof(crc), "*%04X", crc16(few));
    EXPECT_EQ(decodeRobotLine(few + crc).error, DecodeError::kMalformed);
}

TEST(PathLine, RoundTripAndKnownText) {
    const std::string line = encode(point(3, 12, -250, 1500));
    EXPECT_EQ(line.substr(0, 31), "SGT1 5A3C19E2 7 3 12 -250 1500*");
    const PathDecodeResult decoded = decodePathLine(body(line));
    EXPECT_EQ(decoded.error, DecodeError::kNone);
    EXPECT_TRUE(decoded.record == point(3, 12, -250, 1500));
    EXPECT_EQ(lineKind(line), LineKind::kPath);
    EXPECT_EQ(decodeRobotLine(body(line)).error, DecodeError::kMalformed);
}

TEST(PathLine, EmptyPathAndRanges) {
    const std::string none = encode(point(0, 0, 0, 0));
    EXPECT_EQ(none.substr(0, 24), "SGT1 5A3C19E2 7 0 0 0 0*");
    EXPECT_EQ(decodePathLine(body(none)).record.count, 0);

    EXPECT_FALSE(valid(point(1, 0, 0, 0)));  // index without a path
    EXPECT_FALSE(valid(point(0, 0, 5, 0)));  // a position without a path
    EXPECT_FALSE(valid(point(12, 12, 0, 0))); // index past the end
    EXPECT_TRUE(valid(point(11, 12, 0, 0)));
    EXPECT_TRUE(valid(point(65534, 65535, kMaxCoordinateMm, -kMaxCoordinateMm)));
    EXPECT_FALSE(valid(point(0, 1, kMaxCoordinateMm + 1, 0)));
    PathPointRecord r = point(0, 1, 0, 0);
    r.session         = 0;
    EXPECT_FALSE(valid(r));
}

TEST(LineScannerKinds, HandsOutEveryKindAndSkipsUnknownLetters) {
    PoseRecord pose;
    pose.session = kSession;
    char              buf[kMaxLineLength + 8];
    const std::string pose_line(buf, encodeLine(pose, buf, sizeof(buf)));
    const std::string robot_line = encode(robot());
    const std::string path_line  = encode(point(0, 2, 10, 20));

    LineScanner                    scanner;
    const std::vector<std::string> lines =
        scanAll(scanner, "boot\n" + pose_line + "SGX1 1 2 3*ABCD\n" + robot_line +
                             "noise SG" + path_line + "SGP1 cut by a marker " + robot_line);
    ASSERT_EQ(lines.size(), 4u);
    EXPECT_EQ(lines[0] + "\n", pose_line);
    EXPECT_EQ(lines[1] + "\n", robot_line);
    EXPECT_EQ(lines[2] + "\n", path_line);
    EXPECT_EQ(lines[3] + "\n", robot_line);
    EXPECT_EQ(lineKind(lines[0]), LineKind::kPose);
    EXPECT_EQ(lineKind(lines[1]), LineKind::kRobot);
    EXPECT_EQ(lineKind(lines[2]), LineKind::kPath);
    EXPECT_EQ(lineKind("SGX1 1 2 3*ABCD"), LineKind::kUnknown);
    EXPECT_EQ(lineKind("SGP"), LineKind::kUnknown);
    EXPECT_EQ(scanner.stats().lines, 4u);
    EXPECT_EQ(scanner.stats().truncated, 1u); // the pose cut by the robot marker
}

} // namespace
