// aux_gtest.cpp
// Robot footprint and path lines out of the publisher: sent under the
// session, the path once at full speed then repeated, a new path or clear
// starting over, truncation and rejection, and a writer changing the path
// under the transport.

#include <gtest/gtest.h>

#include <atomic>
#include <limits>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "spectagatr/publisher.h"

namespace
{

using namespace spectagatr;
using protocol::DecodeError;
using protocol::PathPointRecord;
using protocol::RobotRecord;

constexpr SessionId kSession = 0x5A3C19E2;

std::atomic<Milliseconds> g_now{0};

Milliseconds fakeClock() { return g_now.load(); }

struct AuxLine {
    bool            ok   = false;
    char            kind = 0; // 'R', 'T' or 0 when nothing came
    std::string     text;
    RobotRecord     robot;
    PathPointRecord point;
};

AuxLine pullAux(Publisher& publisher, Milliseconds now) {
    char              buffer[kLineCapacity];
    AuxLine           out;
    const std::size_t length = publisher.nextAuxLine(now, buffer, sizeof(buffer));
    if (length == 0) {
        return out;
    }
    out.text.assign(buffer, length);
    const std::string_view body(buffer, length - 1);
    out.kind = body[protocol::kMarkerKindAt];
    if (out.kind == 'R') {
        const protocol::RobotDecodeResult r = protocol::decodeRobotLine(body);
        out.ok                              = r.error == DecodeError::kNone;
        out.robot                           = r.record;
    } else if (out.kind == 'T') {
        const protocol::PathDecodeResult r = protocol::decodePathLine(body);
        out.ok                             = r.error == DecodeError::kNone;
        out.point                          = r.record;
    }
    EXPECT_TRUE(out.ok) << out.text;
    return out;
}

void setPath(Publisher& publisher, const std::vector<Point>& points) {
    publisher.setPath(points.data(), points.size());
}

std::vector<Point> zigzag(std::size_t count) {
    std::vector<Point> points;
    for (std::size_t i = 0; i < count; ++i) {
        points.push_back(Point{0.1 * static_cast<double>(i), i % 2 == 0 ? 0.0 : 0.5});
    }
    return points;
}

class SpectaAuxTest : public ::testing::Test {
  protected:
    void SetUp() override {
        g_now = 1000;
        publisher.begin(kSession);
    }

    Publisher    publisher{&fakeClock};
    Milliseconds now = 1000;
};

TEST_F(SpectaAuxTest, NothingWithoutRobotOrPath) {
    EXPECT_EQ(pullAux(publisher, now).kind, 0);
    char buffer[kLineCapacity];
    EXPECT_EQ(publisher.nextLine(now, buffer, sizeof(buffer)), 0u);
}

TEST_F(SpectaAuxTest, RobotGoesAtOnceThenEveryTwoSeconds) {
    publisher.setRobot(RobotBox{0.17, 0.23, 0.15, 0.15, 0.25});
    AuxLine line = pullAux(publisher, now);
    ASSERT_EQ(line.kind, 'R');
    EXPECT_EQ(line.robot.session, kSession);
    EXPECT_EQ(line.robot.front_mm, 170);
    EXPECT_EQ(line.robot.back_mm, 230);
    EXPECT_EQ(line.robot.height_mm, 250);
    EXPECT_EQ(line.text.substr(0, 31), "SGR1 5A3C19E2 170 230 150 150 2");

    EXPECT_EQ(pullAux(publisher, now + 1999).kind, 0);
    line = pullAux(publisher, now + kRobotRepeatMs);
    EXPECT_EQ(line.kind, 'R');
    EXPECT_EQ(publisher.stats().robot_lines, 2u);

    // A new box goes at once.
    publisher.setRobot(RobotBox::centered(0.4, 0.3, 0.2));
    line = pullAux(publisher, now + kRobotRepeatMs + 1);
    ASSERT_EQ(line.kind, 'R');
    EXPECT_EQ(line.robot.front_mm, 200);
    EXPECT_EQ(line.robot.left_mm, 150);
    EXPECT_EQ(line.robot.height_mm, 200);

    // A new session sends it again.
    publisher.begin(kSession + 1);
    line = pullAux(publisher, now + kRobotRepeatMs + 2);
    ASSERT_EQ(line.kind, 'R');
    EXPECT_EQ(line.robot.session, kSession + 1);
}

TEST_F(SpectaAuxTest, RobotRejectsNoSizeAndNonFinite) {
    publisher.setRobot(RobotBox{});
    publisher.setRobot(RobotBox{0.1, 0.1, 0, 0, 0.1});
    publisher.setRobot(RobotBox{0.1, 0.1, 0.1, 0.1, std::numeric_limits<double>::quiet_NaN()});
    publisher.setRobot(RobotBox{kMaxRobotSize + 0.001, 0.1, 0.1, 0.1, 0.1});
    EXPECT_EQ(publisher.stats().rejected, 4u);
    EXPECT_EQ(pullAux(publisher, now).kind, 0);
}

TEST_F(SpectaAuxTest, PathGoesOnceAtFullSpeedThenRepeats) {
    const std::vector<Point> points = zigzag(4);
    setPath(publisher,points);
    EXPECT_EQ(publisher.stats().paths, 1u);

    // First pass: every call gives the next point, whatever the clock says.
    for (std::size_t i = 0; i < 4; ++i) {
        const AuxLine line = pullAux(publisher, now);
        ASSERT_EQ(line.kind, 'T') << i;
        EXPECT_EQ(line.point.session, kSession);
        EXPECT_EQ(line.point.path_id, 1);
        EXPECT_EQ(line.point.index, i);
        EXPECT_EQ(line.point.count, 4);
        EXPECT_EQ(line.point.x_mm, static_cast<int32_t>(100 * i));
        EXPECT_EQ(line.point.y_mm, i % 2 == 0 ? 0 : 500);
    }
    EXPECT_EQ(publisher.stats().path_lines, 4u);

    // Then one point per kPathRepeatMs / count, from index 0 again.
    EXPECT_EQ(pullAux(publisher, now).kind, 0);
    const Milliseconds spacing = kPathRepeatMs / 4;
    EXPECT_EQ(pullAux(publisher, now + spacing - 1).kind, 0);
    AuxLine line = pullAux(publisher, now + spacing);
    ASSERT_EQ(line.kind, 'T');
    EXPECT_EQ(line.point.index, 0);
    EXPECT_EQ(pullAux(publisher, now + spacing).kind, 0);
    line = pullAux(publisher, now + 2 * spacing);
    ASSERT_EQ(line.kind, 'T');
    EXPECT_EQ(line.point.index, 1);
}

TEST_F(SpectaAuxTest, LongPathRepeatsNoFasterThanTheMinimumSpacing) {
    setPath(publisher,zigzag(200));
    for (std::size_t i = 0; i < 200; ++i) {
        ASSERT_EQ(pullAux(publisher, now).point.index, i);
    }
    EXPECT_EQ(pullAux(publisher, now + kPathMinSpacingMs - 1).kind, 0);
    EXPECT_EQ(pullAux(publisher, now + kPathMinSpacingMs).point.index, 0);
}

TEST_F(SpectaAuxTest, NewPathAndClearStartOverWithANewId) {
    setPath(publisher,zigzag(6));
    ASSERT_EQ(pullAux(publisher, now).point.index, 0);
    ASSERT_EQ(pullAux(publisher, now).point.index, 1);

    setPath(publisher,zigzag(3)); // shorter, mid pass
    AuxLine line = pullAux(publisher, now);
    ASSERT_EQ(line.kind, 'T');
    EXPECT_EQ(line.point.path_id, 2);
    EXPECT_EQ(line.point.index, 0);
    EXPECT_EQ(line.point.count, 3);
    ASSERT_EQ(pullAux(publisher, now).point.index, 1);
    ASSERT_EQ(pullAux(publisher, now).point.index, 2);

    publisher.clearPath();
    line = pullAux(publisher, now);
    ASSERT_EQ(line.kind, 'T');
    EXPECT_EQ(line.point.path_id, 3);
    EXPECT_EQ(line.point.count, 0);
    EXPECT_EQ(line.point.index, 0);
    EXPECT_EQ(line.text.substr(0, 24), "SGT1 5A3C19E2 3 0 0 0 0*");
    // "No path" repeats slowly.
    EXPECT_EQ(pullAux(publisher, now + kPathRepeatMs - 1).kind, 0);
    EXPECT_EQ(pullAux(publisher, now + kPathRepeatMs).point.count, 0);
    EXPECT_EQ(publisher.stats().paths, 3u);
}

TEST_F(SpectaAuxTest, PathTruncatedAtTheCapacityAndRejectedWhenBad) {
    setPath(publisher,zigzag(kMaxPathPoints + 10));
    EXPECT_EQ(publisher.stats().path_truncated, 10u);
    for (std::size_t i = 0; i < kMaxPathPoints; ++i) {
        const AuxLine line = pullAux(publisher, now);
        ASSERT_EQ(line.point.count, kMaxPathPoints);
        ASSERT_EQ(line.point.index, i);
    }

    std::vector<Point> bad = zigzag(3);
    bad[1].y               = std::numeric_limits<double>::infinity();
    setPath(publisher,bad);
    bad[1].y = 0;
    bad[2].x = kMaxCoordinate + 1;
    setPath(publisher,bad);
    EXPECT_EQ(publisher.stats().rejected, 2u);
    EXPECT_EQ(publisher.stats().paths, 1u); // the long path stays
    EXPECT_EQ(pullAux(publisher, now + kPathMinSpacingMs).point.count, kMaxPathPoints);
}

TEST_F(SpectaAuxTest, RobotComesBeforePathAndNewSessionResendsBoth) {
    setPath(publisher,zigzag(2));
    publisher.setRobot(RobotBox::centered(0.4, 0.3, 0.2));
    EXPECT_EQ(pullAux(publisher, now).kind, 'R');
    EXPECT_EQ(pullAux(publisher, now).kind, 'T');
    EXPECT_EQ(pullAux(publisher, now).kind, 'T');
    EXPECT_EQ(pullAux(publisher, now).kind, 0);

    publisher.begin(kSession + 7);
    AuxLine line = pullAux(publisher, now);
    ASSERT_EQ(line.kind, 'R');
    EXPECT_EQ(line.robot.session, kSession + 7);
    line = pullAux(publisher, now);
    ASSERT_EQ(line.kind, 'T');
    EXPECT_EQ(line.point.session, kSession + 7);
    EXPECT_EQ(line.point.index, 0);
    EXPECT_EQ(line.point.path_id, 1); // the path itself did not change
}

TEST_F(SpectaAuxTest, PoseLinesAreUnaffected) {
    setPath(publisher,zigzag(5));
    publisher.updatePose(1.0, 2.0, 0.5);
    char buffer[kLineCapacity];
    EXPECT_GT(publisher.nextLine(now, buffer, sizeof(buffer)), 0u);
    EXPECT_EQ(protocol::lineKind(std::string_view(buffer, 5)), protocol::LineKind::kPose);
    EXPECT_EQ(publisher.stats().lines, 1u);
    EXPECT_EQ(publisher.stats().path_lines, 0u);
}

// One task replaces the path while the transport walks it: every line
// decodes, carries a consistent (id, count) pair, and each point's x and y
// belong to the same path (x encodes the path number, y the index).
TEST(SpectaAuxConcurrency, PathWriterUnderTheTransport) {
    Publisher publisher(&fakeClock);
    publisher.begin(kSession);
    std::atomic<bool> stop{false};
    std::thread       writer([&] {
        std::vector<Point> points;
        for (uint32_t n = 1; !stop.load(); ++n) {
            const std::size_t count = 1 + n % 40;
            points.clear();
            for (std::size_t i = 0; i < count; ++i) {
                points.push_back(Point{static_cast<double>(n), static_cast<double>(i)});
            }
            setPath(publisher,points);
        }
    });
    // The writer keeps the slot busy much of the time; the transport only has
    // to stay consistent, so it runs until it has seen enough lines.
    std::size_t lines = 0, bad = 0;
    for (Milliseconds k = 0; lines < 500 && k < 5000000; ++k) {
        const AuxLine line = pullAux(publisher, k);
        if (line.kind != 'T') {
            continue;
        }
        ++lines;
        const PathPointRecord& p = line.point;
        if (p.index >= p.count || p.y_mm != static_cast<int32_t>(p.index) * 1000 ||
            p.x_mm % 1000 != 0) {
            ++bad;
        }
    }
    stop = true;
    writer.join();
    EXPECT_EQ(bad, 0u);
    EXPECT_EQ(lines, 500u);
}

} // namespace
