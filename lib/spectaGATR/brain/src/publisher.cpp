// publisher.cpp

#include "spectagatr/publisher.h"

#include <cmath>

namespace spectagatr
{
namespace
{

// Pose slot words.
constexpr std::size_t kX            = 0; // mm
constexpr std::size_t kY            = 1; // mm
constexpr std::size_t kYaw          = 2; // centidegrees
constexpr std::size_t kRoll         = 3;
constexpr std::size_t kPitch        = 4;
constexpr std::size_t kTime         = 5; // publisher ms
constexpr std::size_t kEpochFlags   = 6; // epoch in bits 0..15, flags in bits 16..23
constexpr std::size_t kChecksum     = 7;
constexpr std::size_t kDataWords    = 7;
constexpr int         kReadAttempts = 3;

// Robot slot words.
constexpr std::size_t kFront  = 0;
constexpr std::size_t kBack   = 1;
constexpr std::size_t kLeft   = 2;
constexpr std::size_t kRight  = 3;
constexpr std::size_t kHeight = 4;

uint64_t splitmix64(uint64_t x) {
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}

// FNV-1a over the data words. Catches a slot mixed by two overlapping writers.
uint32_t checksum(const uint32_t* words) {
    uint32_t hash = 0x811C9DC5u;
    for (std::size_t i = 0; i < kDataWords; ++i) {
        uint32_t word = words[i];
        for (int byte = 0; byte < 4; ++byte) {
            hash ^= word & 0xFFu;
            hash *= 0x01000193u;
            word >>= 8;
        }
    }
    return hash;
}

bool validCoordinate(Meters v) { return std::isfinite(v) && std::fabs(v) <= kMaxCoordinate; }

bool validSize(Meters v) { return std::isfinite(v) && v >= 0 && v <= kMaxRobotSize; }

uint32_t periodFor(Hertz rate) {
    if (!std::isfinite(rate)) {
        rate = kDefaultRateHz;
    }
    if (rate < kMinRateHz) {
        rate = kMinRateHz;
    }
    if (rate > kMaxRateHz) {
        rate = kMaxRateHz;
    }
    return static_cast<uint32_t>(std::lround(1000.0 / rate));
}

uint32_t toWord(int32_t v) { return static_cast<uint32_t>(v); }

int32_t fromWord(uint32_t w) { return static_cast<int32_t>(w); }

// Seqlock writer entry: odd while inside. A writer that never finished (a
// deleted task) leaves it odd; the next writer finishes it.
void enterWrite(std::atomic<uint32_t>& seq) {
    seq.fetch_or(1u, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
}

void leaveWrite(std::atomic<uint32_t>& seq) {
    uint32_t value = seq.load(std::memory_order_relaxed);
    while (!seq.compare_exchange_weak(value, (value | 1u) + 1u, std::memory_order_release,
                                      std::memory_order_relaxed)) {
    }
}

} // namespace

SessionId makeSessionId(const uint64_t* inputs, std::size_t count) {
    uint64_t hash = splitmix64(count);
    for (std::size_t i = 0; i < count; ++i) {
        hash = splitmix64(hash ^ inputs[i]);
    }
    const SessionId id = static_cast<SessionId>(hash ^ (hash >> 32));
    return id != 0 ? id : 1;
}

Publisher::Publisher(Clock clock, Hertz rate) : clock_(clock) {
    period_ms_.store(periodFor(rate), std::memory_order_relaxed);
}

// --- pose ---

void Publisher::updatePose(Meters x, Meters y, Radians heading) {
    store(x, y, Attitude{0, 0, heading}, false);
}

void Publisher::updatePose(Meters x, Meters y, const Attitude& attitude) {
    store(x, y, attitude, true);
}

void Publisher::store(Meters x, Meters y, const Attitude& attitude, bool full_attitude) {
    if (!validCoordinate(x) || !validCoordinate(y) || !std::isfinite(attitude.roll) ||
        !std::isfinite(attitude.pitch) || !std::isfinite(attitude.yaw)) {
        rejected_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    const uint32_t flags = full_attitude ? static_cast<uint32_t>(protocol::kFullAttitude) : 0u;
    const uint32_t epoch = epoch_.load(std::memory_order_relaxed) & 0xFFFFu;

    uint32_t words[kSlotWords];
    words[kX]          = toWord(protocol::toMillimeters(x));
    words[kY]          = toWord(protocol::toMillimeters(y));
    words[kYaw]        = toWord(protocol::toCentidegrees(attitude.yaw));
    words[kRoll]       = toWord(protocol::toCentidegrees(attitude.roll));
    words[kPitch]      = toWord(protocol::toCentidegrees(attitude.pitch));
    words[kTime]       = clock_ != nullptr ? clock_() : 0u;
    words[kEpochFlags] = epoch | (flags << 16);
    words[kChecksum]   = checksum(words);

    enterWrite(slot_seq_);
    for (std::size_t i = 0; i < kSlotWords; ++i) {
        slot_[i].store(words[i], std::memory_order_relaxed);
    }
    leaveWrite(slot_seq_);
    updates_.fetch_add(1, std::memory_order_relaxed);
}

bool Publisher::load(uint32_t& version, uint32_t (&words)[kSlotWords]) const {
    for (int attempt = 0; attempt < kReadAttempts; ++attempt) {
        const uint32_t before = slot_seq_.load(std::memory_order_acquire);
        if (before == 0) {
            return false; // never written
        }
        if ((before & 1u) != 0) {
            continue;
        }
        for (std::size_t i = 0; i < kSlotWords; ++i) {
            words[i] = slot_[i].load(std::memory_order_relaxed);
        }
        std::atomic_thread_fence(std::memory_order_acquire);
        if (slot_seq_.load(std::memory_order_relaxed) != before ||
            checksum(words) != words[kChecksum]) {
            continue;
        }
        version = before;
        return true;
    }
    return false;
}

void Publisher::resetTrail() { epoch_.fetch_add(1, std::memory_order_relaxed); }

// --- robot ---

void Publisher::setRobot(const RobotBox& box) {
    if (!validSize(box.front) || !validSize(box.back) || !validSize(box.left) ||
        !validSize(box.right) || !validSize(box.height)) {
        rejected_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    uint32_t words[kRobotWords];
    words[kFront]  = toWord(protocol::toMillimeters(box.front));
    words[kBack]   = toWord(protocol::toMillimeters(box.back));
    words[kLeft]   = toWord(protocol::toMillimeters(box.left));
    words[kRight]  = toWord(protocol::toMillimeters(box.right));
    words[kHeight] = toWord(protocol::toMillimeters(box.height));
    protocol::RobotRecord record;
    record.session   = 1; // the shape check only
    record.front_mm  = fromWord(words[kFront]);
    record.back_mm   = fromWord(words[kBack]);
    record.left_mm   = fromWord(words[kLeft]);
    record.right_mm  = fromWord(words[kRight]);
    record.height_mm = fromWord(words[kHeight]);
    if (!protocol::valid(record)) {
        rejected_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    enterWrite(robot_seq_);
    for (std::size_t i = 0; i < kRobotWords; ++i) {
        robot_[i].store(words[i], std::memory_order_relaxed);
    }
    leaveWrite(robot_seq_);
}

bool Publisher::loadRobot(uint32_t& version, protocol::RobotRecord& record) const {
    for (int attempt = 0; attempt < kReadAttempts; ++attempt) {
        const uint32_t before = robot_seq_.load(std::memory_order_acquire);
        if (before == 0) {
            return false;
        }
        if ((before & 1u) != 0) {
            continue;
        }
        uint32_t words[kRobotWords];
        for (std::size_t i = 0; i < kRobotWords; ++i) {
            words[i] = robot_[i].load(std::memory_order_relaxed);
        }
        std::atomic_thread_fence(std::memory_order_acquire);
        if (robot_seq_.load(std::memory_order_relaxed) != before) {
            continue;
        }
        record.front_mm  = fromWord(words[kFront]);
        record.back_mm   = fromWord(words[kBack]);
        record.left_mm   = fromWord(words[kLeft]);
        record.right_mm  = fromWord(words[kRight]);
        record.height_mm = fromWord(words[kHeight]);
        version          = before;
        return true;
    }
    return false;
}

// --- path ---

bool Publisher::validPoint(const Point& point) {
    return validCoordinate(point.x) && validCoordinate(point.y);
}

void Publisher::setPath(const Point* points, std::size_t count) {
    if (points == nullptr) {
        count = 0;
    }
    setPath(count, [points](std::size_t i) { return points[i]; });
}

void Publisher::clearPath() { setPath(nullptr, 0); }

void Publisher::beginPath() { enterWrite(path_seq_); }

void Publisher::writePathPoint(std::size_t index, const Point& point) {
    path_x_[index].store(toWord(protocol::toMillimeters(point.x)), std::memory_order_relaxed);
    path_y_[index].store(toWord(protocol::toMillimeters(point.y)), std::memory_order_relaxed);
}

void Publisher::endPath(std::size_t count) {
    const uint32_t id = (path_id_.fetch_add(1, std::memory_order_relaxed) + 1) & 0xFFFFu;
    path_meta_.store(static_cast<uint32_t>(count) | (id << 16), std::memory_order_relaxed);
    leaveWrite(path_seq_);
    paths_.fetch_add(1, std::memory_order_relaxed);
}

bool Publisher::loadPathPoint(std::size_t index, uint32_t& version,
                              protocol::PathPointRecord& record) const {
    for (int attempt = 0; attempt < kReadAttempts; ++attempt) {
        const uint32_t before = path_seq_.load(std::memory_order_acquire);
        if (before == 0) {
            return false;
        }
        if ((before & 1u) != 0) {
            continue;
        }
        const uint32_t meta  = path_meta_.load(std::memory_order_relaxed);
        const uint32_t count = meta & 0xFFFFu;
        uint32_t       x = 0, y = 0;
        if (index < count) {
            x = path_x_[index].load(std::memory_order_relaxed);
            y = path_y_[index].load(std::memory_order_relaxed);
        }
        std::atomic_thread_fence(std::memory_order_acquire);
        if (path_seq_.load(std::memory_order_relaxed) != before) {
            continue;
        }
        if (count > 0 && index >= count) {
            return false; // the path shrank under the cursor
        }
        record.path_id = static_cast<PathId>(meta >> 16);
        record.count   = static_cast<protocol::PathIndex>(count);
        record.index   = static_cast<protocol::PathIndex>(count > 0 ? index : 0);
        record.x_mm    = fromWord(x);
        record.y_mm    = fromWord(y);
        version        = before;
        return true;
    }
    return false;
}

// --- rate and stats ---

void Publisher::setRate(Hertz rate) {
    period_ms_.store(periodFor(rate), std::memory_order_relaxed);
}

Milliseconds Publisher::period() const { return period_ms_.load(std::memory_order_relaxed); }

Hertz Publisher::rate() const { return 1000.0 / static_cast<double>(period()); }

PublisherStats Publisher::stats() const {
    PublisherStats s;
    s.updates        = updates_.load(std::memory_order_relaxed);
    s.rejected       = rejected_.load(std::memory_order_relaxed);
    s.lines          = lines_.load(std::memory_order_relaxed);
    s.overwritten    = overwritten_.load(std::memory_order_relaxed);
    s.write_errors   = write_errors_.load(std::memory_order_relaxed);
    s.robot_lines    = robot_lines_.load(std::memory_order_relaxed);
    s.path_lines     = path_lines_.load(std::memory_order_relaxed);
    s.paths          = paths_.load(std::memory_order_relaxed);
    s.path_truncated = path_truncated_.load(std::memory_order_relaxed);
    return s;
}

SessionId Publisher::session() const { return session_.load(std::memory_order_relaxed); }

// --- transport ---

void Publisher::begin(SessionId session, Sequence first_seq) {
    next_seq_           = first_seq;
    scheduled_          = false;
    robot_sent_version_ = 0;
    robot_scheduled_    = false;
    path_sent_version_  = 0;
    path_cursor_        = 0;
    path_first_pass_    = true;
    path_scheduled_     = false;
    session_.store(session, std::memory_order_relaxed);
}

std::size_t Publisher::nextLine(Milliseconds now, char* out, std::size_t cap) {
    const SessionId session = session_.load(std::memory_order_relaxed);
    if (session == 0 || out == nullptr || cap < kLineCapacity) {
        return 0;
    }
    if (scheduled_ && static_cast<int32_t>(now - due_) < 0) {
        return 0;
    }
    uint32_t version = 0;
    uint32_t words[kSlotWords];
    if (!load(version, words) || version == sent_version_) {
        return 0;
    }

    protocol::PoseRecord record;
    record.session           = session;
    record.epoch             = static_cast<Epoch>(words[kEpochFlags] & 0xFFFFu);
    record.seq               = next_seq_;
    record.t_ms              = words[kTime];
    record.x_mm              = fromWord(words[kX]);
    record.y_mm              = fromWord(words[kY]);
    record.yaw_cdeg          = fromWord(words[kYaw]);
    record.roll_cdeg         = fromWord(words[kRoll]);
    record.pitch_cdeg        = fromWord(words[kPitch]);
    record.flags             = static_cast<uint8_t>(words[kEpochFlags] >> 16);
    const std::size_t length = protocol::encodeLine(record, out, cap);
    if (length == 0) {
        return 0;
    }

    // Each completed write adds 2 to the slot version.
    const uint32_t supplied = (version - sent_version_) / 2;
    if (supplied > 1) {
        overwritten_.fetch_add(supplied - 1, std::memory_order_relaxed);
    }
    sent_version_ = version;
    ++next_seq_;
    lines_.fetch_add(1, std::memory_order_relaxed);

    // Lines on a fixed grid; more than a period behind starts it over.
    const Milliseconds spacing = period();
    if (!scheduled_) {
        due_       = now + spacing;
        scheduled_ = true;
    } else {
        due_ += spacing;
        if (static_cast<int32_t>(now - due_) >= 0) {
            due_ = now + spacing;
        }
    }
    return length;
}

std::size_t Publisher::nextAuxLine(Milliseconds now, char* out, std::size_t cap) {
    if (session_.load(std::memory_order_relaxed) == 0 || out == nullptr || cap < kLineCapacity) {
        return 0;
    }
    const std::size_t robot = robotLine(now, out, cap);
    return robot > 0 ? robot : pathLine(now, out, cap);
}

std::size_t Publisher::robotLine(Milliseconds now, char* out, std::size_t cap) {
    uint32_t              version = 0;
    protocol::RobotRecord record;
    if (!loadRobot(version, record)) {
        return 0;
    }
    const bool changed = version != robot_sent_version_;
    const bool due     = !robot_scheduled_ || static_cast<int32_t>(now - robot_due_) >= 0;
    if (!changed && !due) {
        return 0;
    }
    record.session           = session_.load(std::memory_order_relaxed);
    const std::size_t length = protocol::encodeRobotLine(record, out, cap);
    if (length == 0) {
        return 0;
    }
    robot_sent_version_ = version;
    robot_scheduled_    = true;
    robot_due_          = now + kRobotRepeatMs;
    robot_lines_.fetch_add(1, std::memory_order_relaxed);
    return length;
}

std::size_t Publisher::pathLine(Milliseconds now, char* out, std::size_t cap) {
    uint32_t                  version = 0;
    protocol::PathPointRecord record;
    if (!loadPathPoint(path_cursor_, version, record)) {
        // A path that shrank under the cursor: start it over.
        if (path_sent_version_ != 0 && loadPathPoint(0, version, record)) {
            path_cursor_ = 0;
        } else {
            return 0;
        }
    }
    if (version != path_sent_version_) {
        // A new path: from its first point, as fast as the output allows.
        path_sent_version_ = version;
        path_first_pass_   = true;
        path_scheduled_    = false;
        if (path_cursor_ != 0 && !loadPathPoint(0, version, record)) {
            return 0;
        }
        path_cursor_ = 0;
    }
    if (!path_first_pass_ && path_scheduled_ && static_cast<int32_t>(now - path_due_) < 0) {
        return 0;
    }
    record.session           = session_.load(std::memory_order_relaxed);
    const std::size_t length = protocol::encodePathLine(record, out, cap);
    if (length == 0) {
        return 0;
    }
    const std::size_t points = record.count > 0 ? record.count : 1;
    if (++path_cursor_ >= points) {
        path_cursor_     = 0;
        path_first_pass_ = false;
    }
    Milliseconds spacing = kPathRepeatMs / static_cast<Milliseconds>(points);
    if (spacing < kPathMinSpacingMs) {
        spacing = kPathMinSpacingMs;
    }
    path_due_       = now + (path_first_pass_ ? 0 : spacing);
    path_scheduled_ = true;
    path_lines_.fetch_add(1, std::memory_order_relaxed);
    return length;
}

void Publisher::recordWrite(bool ok) {
    if (!ok) {
        write_errors_.fetch_add(1, std::memory_order_relaxed);
    }
}

} // namespace spectagatr
