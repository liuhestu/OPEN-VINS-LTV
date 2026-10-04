#include "ltv/LtvIdentityGuard.h"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <vector>

int main() {
  using Guard = ltv::LtvIdentityGuard;
  static_assert(sizeof(Guard::Words) == 1048576, "Bloom storage must be exactly 1MiB");
  static_assert(sizeof(Guard) <= 1048576 + 32, "Guard metadata must stay bounded");
  auto guard = std::unique_ptr<Guard>(new Guard(7));
  const auto *storage = guard->words().data();
  assert(!guard->potentiallySeen(0));
  std::vector<uint64_t> ids;
  for (uint64_t i = 0; i < 100000; ++i)
    ids.push_back(i * UINT64_C(0x9e3779b97f4a7c15));
  std::mt19937 order(42);
  std::shuffle(ids.begin(), ids.end(), order);
  for (auto id : ids) {
    const size_t old_bits = guard->setBitCount();
    guard->insert(id);
    assert(guard->potentiallySeen(id));
    assert(guard->setBitCount() >= old_bits);
    assert(guard->words().data() == storage);
  }
  for (auto id : ids)
    assert(guard->potentiallySeen(id));
  guard->insert(std::numeric_limits<uint64_t>::max());
  assert(guard->potentiallySeen(std::numeric_limits<uint64_t>::max()));
  const auto occupied = guard->setBitCount();
  std::reverse(ids.begin(), ids.end());
  for (auto id : ids)
    guard->insert(id);
  assert(guard->setBitCount() == occupied);
  assert(guard->insertionCalls() == 200001);
  assert(guard->occupancyFraction() > 0 && guard->occupancyFraction() <= 1);
  auto copy = std::unique_ptr<Guard>(new Guard(*guard));
  assert(copy->words().data() != guard->words().data());
  assert(copy->words() == guard->words());
  copy->reset(8);
  assert(copy->epoch() == 8 && copy->setBitCount() == 0 && copy->insertionCalls() == 0);
  assert(!copy->potentiallySeen(ids[0]) && guard->potentiallySeen(ids[0]));
  copy->insert(123);
  assert(guard->setBitCount() == occupied);
  *copy = *guard;
  assert(copy->words() == guard->words() && copy->epoch() == guard->epoch());
  copy->reset(8);
  assert(copy->setBitCount() == 0 && guard->setBitCount() == occupied);
  copy->insert(std::numeric_limits<uint64_t>::max());
  bool rejected = false;
  try {
    guard->reset(7);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  assert(rejected && guard->potentiallySeen(ids[0]));
  rejected = false;
  try {
    copy->reset(6);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  assert(rejected);
  // All hash positions are bounded and distinct for each ID (odd stride mod2^23).
  for (auto id : {UINT64_C(0), UINT64_C(1), UINT64_C(0xffffffffffffffff)}) {
    auto positions = Guard::bitPositions(id);
    for (auto bit : positions)
      assert(bit < Guard::capacityBits());
    std::sort(positions.begin(), positions.end());
    assert(std::adjacent_find(positions.begin(), positions.end()) == positions.end());
  }
  std::cout << "PASS 100000 arbitrary-order IDs no false negatives; duplicates/copy/epoch reset/fixed capacity. "
            << "Insertion calls=" << guard->insertionCalls() << " set bits=" << guard->setBitCount()
            << " occupancy=" << guard->occupancyFraction() << "\n";
}
