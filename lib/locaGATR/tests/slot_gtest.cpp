// slot_gtest.cpp

#include "locagatr/slot.h"

#include <gtest/gtest.h>

#include <atomic>
#include <thread>

namespace locagatr
{
namespace
{

// Every field derived from one number, so a torn copy is detectable.
struct Sample {
    uint32_t n = 0;
    double   a = 0;
    double   b = 0;
    bool     odd = false;
};

Sample make(uint32_t n) { return Sample{n, n * 0.5, n * -2.0, (n & 1u) != 0}; }

bool consistent(const Sample& s) {
    return s.a == s.n * 0.5 && s.b == s.n * -2.0 && s.odd == ((s.n & 1u) != 0);
}

TEST(Slot, EmptyThenNewest) {
    Slot<Sample> slot;
    Sample       out;
    EXPECT_FALSE(slot.load(out));
    const uint32_t v0 = slot.version();
    slot.store(make(1));
    slot.store(make(2));
    ASSERT_TRUE(slot.load(out));
    EXPECT_EQ(out.n, 2u);
    EXPECT_NE(slot.version(), v0);
}

TEST(Slot, ReaderNeverSeesATornValue) {
    Slot<Sample>      slot;
    std::atomic<bool> done{false};
    std::thread       writer([&] {
        for (uint32_t n = 1; n <= 200000; ++n) {
            slot.store(make(n));
        }
        done = true;
    });
    bool     ok   = true;
    uint32_t last = 0;
    while (!done) {
        Sample s;
        if (slot.load(s)) {
            ok   = ok && consistent(s) && s.n >= last;
            last = s.n;
        }
    }
    writer.join();
    EXPECT_TRUE(ok);
}

} // namespace
} // namespace locagatr
