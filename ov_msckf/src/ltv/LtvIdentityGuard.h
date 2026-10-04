#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace ltv {
// Fixed-space, non-deleting identity protection for one explicit epoch.
// A positive query may be a collision; it is never evidence of a true repeat.
// No unique-ID count or calibrated false-positive probability is asserted.
class LtvIdentityGuard {
public:
  static constexpr size_t capacityBytes() { return 1u << 20; }
  static constexpr size_t capacityBits() { return capacityBytes() * 8; }
  static constexpr unsigned int hashCount() { return 7; }
  static constexpr uint32_t formatVersion() { return 1; }
  using Words = std::array<uint64_t, (1u << 20) / sizeof(uint64_t)>;

  explicit LtvIdentityGuard(uint64_t epoch = 0) : epoch_(epoch) {
    static_assert(sizeof(size_t) <= sizeof(uint64_t), "Feature size_t IDs must map losslessly to uint64_t");
    static_assert(sizeof(uint64_t) == 8, "Identity guard requires 64-bit unsigned words");
  }

  bool potentiallySeen(uint64_t id) const {
    const auto positions = bitPositions(id);
    for (auto bit : positions)
      if (!(words_[bit >> 6] & (UINT64_C(1) << (bit & 63))))
        return false;
    return true;
  }

  // Call only after the owning transaction succeeds. Duplicate insertion is
  // harmless but increments insertionCalls(), which is intentionally not unique.
  void insert(uint64_t id) {
    if (insertion_calls_ == std::numeric_limits<uint64_t>::max())
      throw std::overflow_error("Identity guard insertion counter exhausted");
    const auto positions = bitPositions(id);
    for (auto bit : positions) {
      auto &word = words_[bit >> 6];
      const uint64_t mask = UINT64_C(1) << (bit & 63);
      if (!(word & mask)) {
        word |= mask;
        ++set_bits_;
      }
    }
    ++insertion_calls_;
  }

  // A populated epoch cannot be silently cleared or rewound. The caller must
  // commit a real observer-epoch transition, never reset to reduce occupancy.
  void reset(uint64_t new_epoch) {
    if (new_epoch < epoch_ || (insertion_calls_ && new_epoch == epoch_))
      throw std::invalid_argument("Identity guard reset requires a newer populated epoch");
    words_.fill(0);
    epoch_ = new_epoch;
    insertion_calls_ = 0;
    set_bits_ = 0;
  }

  uint64_t epoch() const { return epoch_; }
  uint64_t insertionCalls() const { return insertion_calls_; }
  size_t setBitCount() const { return set_bits_; }
  double occupancyFraction() const { return static_cast<double>(set_bits_) / capacityBits(); }
  const Words &words() const { return words_; }

  // Version-1 hash contract: defined unsigned64 arithmetic (including wrap),
  // explicit salts, no std::hash, host byte order, pointer value or random seed.
  static std::array<size_t, 7> bitPositions(uint64_t id) {
    const uint64_t first = mix(id ^ UINT64_C(0x243f6a8885a308d3));
    const uint64_t stride = mix(id ^ UINT64_C(0x13198a2e03707344)) | UINT64_C(1);
    std::array<size_t, 7> result{};
    for (unsigned int j = 0; j < hashCount(); ++j)
      result[j] = static_cast<size_t>((first + j * stride) & UINT64_C(0x7fffff));
    return result;
  }

private:
  static uint64_t mix(uint64_t value) {
    value = (value ^ (value >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
    value = (value ^ (value >> 27)) * UINT64_C(0x94d049bb133111eb);
    return value ^ (value >> 31);
  }
  Words words_{};
  uint64_t epoch_ = 0, insertion_calls_ = 0;
  size_t set_bits_ = 0;
};
} // namespace ltv
