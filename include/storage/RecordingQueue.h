#pragma once
#include <array>
#include <atomic>
#include <cstdint>

namespace storage {
// Exactly one producer and one consumer; reset only after consumer ownership ends.
template<typename T, uint32_t Capacity> class RecordingQueue {
  static_assert(Capacity && !(Capacity & (Capacity-1)), "Power-of-two queue capacity required");
public:
  bool push(const T &value, uint32_t &occupancy) {
    const auto head = head_.load(std::memory_order_relaxed);
    const auto tail = tail_.load(std::memory_order_acquire);
    if (head-tail == Capacity) return false;
    values_[head & (Capacity-1)] = value; occupancy = head-tail+1;
    head_.store(head+1, std::memory_order_release); return true;
  }
  bool pop(T &value) {
    const auto tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) return false;
    value = values_[tail & (Capacity-1)]; tail_.store(tail+1, std::memory_order_release); return true;
  }
  uint32_t size() const { // Producer-side inspection.
    return head_.load(std::memory_order_relaxed)-tail_.load(std::memory_order_acquire);
  }
  bool empty() const { // Consumer-side inspection.
    return tail_.load(std::memory_order_relaxed) == head_.load(std::memory_order_acquire);
  }
  void reset() { head_.store(0); tail_.store(0); }
private:
  std::array<T, Capacity> values_;
  std::atomic<uint32_t> head_{0}, tail_{0};
};
}
