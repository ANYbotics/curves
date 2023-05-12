/*
 * KeyGenerator.cpp
 *
 *  Created on: Aug 19, 2014
 *      Author: Paul Furgale, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <boost/thread.hpp>
#include <curves/KeyGenerator.hpp>

namespace curves {

size_t KeyGenerator::getNextKey() {
  static size_t key = 0;
  static boost::mutex mutex;
  boost::lock_guard<boost::mutex> guard(mutex);
  return ++key;
}

}  // namespace curves
