#include "curves/KeyGenerator.hpp"

#include <atomic>

namespace curves {

size_t KeyGenerator::getNextKey() {
  static_assert(std::atomic<size_t>::is_always_lock_free);
  static std::atomic<size_t> key(0);
  return ++key;
}

}  // namespace curves
