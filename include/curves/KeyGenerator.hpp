#pragma once

#include <cstddef>

namespace curves {

class KeyGenerator {
 public:
  static size_t getNextKey();
};

}  // namespace curves
