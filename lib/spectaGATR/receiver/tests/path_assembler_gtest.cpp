// path_assembler_gtest.cpp
// Path points in any order and repeated into one path; a new id, session or count starts over; a
// cleared path; the memory cap.

#include "receiver/feed/path_assembler.h"

#include <gtest/gtest.h>

namespace spectagatr::receiver::feed
{
namespace
{

protocol::PathPointRecord point(protocol::PathIndex index, protocol::PathIndex count,
                                protocol::PathId id = 1, protocol::SessionId session = 0x11) {
    protocol::PathPointRecord r;
    r.session = session;
    r.path_id = id;
    r.index   = index;
    r.count   = count;
    r.x_mm    = 100 * index;
    r.y_mm    = -50 * index;
    return r;
}

TEST(PathAssembler, PointsInAnyOrderRepeatedOnceEach) {
    PathAssembler path;
    EXPECT_FALSE(path.hasPath());
    EXPECT_TRUE(path.offer(point(2, 3)));
    EXPECT_TRUE(path.hasPath());
    EXPECT_EQ(path.count(), 3);
    EXPECT_EQ(path.known(), 1u);
    EXPECT_FALSE(path.complete());
    EXPECT_FALSE(path.points()[0]);
    ASSERT_TRUE(path.points()[2]);
    EXPECT_EQ(path.points()[2]->x_mm, 200);

    EXPECT_FALSE(path.offer(point(2, 3))); // again: nothing new
    EXPECT_TRUE(path.offer(point(0, 3)));
    EXPECT_TRUE(path.offer(point(1, 3)));
    EXPECT_TRUE(path.complete());
    EXPECT_FALSE(path.offer(point(1, 3)));
    EXPECT_EQ(path.known(), 3u);
}

TEST(PathAssembler, NewIdSessionOrCountStartsOver) {
    PathAssembler path;
    path.offer(point(0, 2));
    path.offer(point(1, 2));
    EXPECT_TRUE(path.complete());

    EXPECT_TRUE(path.offer(point(0, 4, 2))); // new path id
    EXPECT_EQ(path.id(), 2);
    EXPECT_EQ(path.count(), 4);
    EXPECT_EQ(path.known(), 1u);

    EXPECT_TRUE(path.offer(point(3, 4, 2, 0x22))); // another publisher run
    EXPECT_EQ(path.session(), 0x22u);
    EXPECT_EQ(path.known(), 1u);
    EXPECT_FALSE(path.points()[0]);

    EXPECT_TRUE(path.offer(point(3, 5, 2, 0x22))); // the same id with another count: start over
    EXPECT_EQ(path.count(), 5);
    EXPECT_EQ(path.known(), 1u);
}

TEST(PathAssembler, ClearedPathAndClear) {
    PathAssembler path;
    path.offer(point(0, 2));
    EXPECT_TRUE(path.offer(point(0, 0, 2))); // the publisher cleared it
    EXPECT_TRUE(path.hasPath());
    EXPECT_EQ(path.count(), 0);
    EXPECT_TRUE(path.complete());
    EXPECT_TRUE(path.points().empty());
    EXPECT_FALSE(path.offer(point(0, 0, 2)));

    EXPECT_TRUE(path.clear());
    EXPECT_FALSE(path.hasPath());
    EXPECT_FALSE(path.clear());
}

TEST(PathAssembler, AMovedPointIsTakenAndTooLongPathsRefused) {
    PathAssembler             path;
    protocol::PathPointRecord p = point(1, 2);
    path.offer(p);
    p.x_mm = 999;
    EXPECT_TRUE(path.offer(p));
    EXPECT_EQ(path.points()[1]->x_mm, 999);

    protocol::PathPointRecord big = point(0, 0);
    big.count                     = static_cast<protocol::PathIndex>(kMaxPathPointsReceived + 1);
    EXPECT_FALSE(path.offer(big));
    EXPECT_EQ(path.refused(), 1u);
    EXPECT_EQ(path.count(), 2); // the shown path stays
    big.count = static_cast<protocol::PathIndex>(kMaxPathPointsReceived);
    EXPECT_TRUE(path.offer(big));
    EXPECT_EQ(path.points().size(), kMaxPathPointsReceived);
}

} // namespace
} // namespace spectagatr::receiver::feed
