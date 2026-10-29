// slot.h
// Latest value of a small plain struct: one writer, any readers, no lock.
// A seqlock, so a task deleted mid read or write leaves nothing locked.
// Portable.

#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace locagatr
{

template <typename T> class Slot {
    static_assert(std::is_trivially_copyable<T>::value, "Slot holds plain data");

  public:
    void store(const T& value) {
        uint32_t words[kWords] = {};
        std::memcpy(words, &value, sizeof(T));
        const uint32_t v = version_.load(std::memory_order_relaxed);
        version_.store(v + 1, std::memory_order_relaxed); // odd: writing
        std::atomic_thread_fence(std::memory_order_release);
        for (std::size_t i = 0; i < kWords; ++i) {
            words_[i].store(words[i], std::memory_order_relaxed);
        }
        version_.store(v + 2, std::memory_order_release);
    }

    // False if nothing stored yet or every try overlapped a write.
    bool load(T& out) const {
        for (int attempt = 0; attempt < 4; ++attempt) {
            const uint32_t before = version_.load(std::memory_order_acquire);
            if (before == 0 || (before & 1u) != 0) {
                continue;
            }
            uint32_t words[kWords];
            for (std::size_t i = 0; i < kWords; ++i) {
                words[i] = words_[i].load(std::memory_order_relaxed);
            }
            std::atomic_thread_fence(std::memory_order_acquire);
            if (version_.load(std::memory_order_relaxed) == before) {
                std::memcpy(&out, words, sizeof(T));
                return true;
            }
        }
        return false;
    }

    // Changes on every store.
    uint32_t version() const { return version_.load(std::memory_order_acquire); }

  private:
    static constexpr std::size_t kWords = (sizeof(T) + 3) / 4;

    std::atomic<uint32_t>                       version_{0};
    std::array<std::atomic<uint32_t>, kWords>   words_{};
};

} // namespace locagatr
