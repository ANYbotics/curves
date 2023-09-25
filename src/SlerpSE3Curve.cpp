#include "curves/SlerpSE3Curve.hpp"

namespace curves {

Time SlerpSE3Curve::getMaxTime() const {
  return manager_.getMaxTime();
}

Time SlerpSE3Curve::getMinTime() const {
  return manager_.getMinTime();
}

bool SlerpSE3Curve::isEmpty() const {
  return manager_.empty();
}

int SlerpSE3Curve::size() const {
  return manager_.size();
}

void SlerpSE3Curve::fitCurve(const std::vector<Time>& times, const std::vector<ValueType>& values, std::vector<Key>* outKeys) {
  assert(times.size() == values.size());
  if (!times.empty()) {
    clear();
    manager_.insertCoefficients(times, values, outKeys);
  }
}

void SlerpSE3Curve::extend(const std::vector<Time>& times, const std::vector<ValueType>& values, std::vector<Key>* outKeys) {
  if (times.size() != values.size()) {
    assert(times.size() == values.size() && "Number of times and number of coefficients don't match.");
  }

  slerpPolicy_.extend<SlerpSE3Curve, ValueType>(times, values, this, outKeys);
}

void SlerpSE3Curve::setMinSamplingPeriod(Time time) {
  slerpPolicy_.setMinSamplingPeriod(time);
}

///   eg. 4 will add a coefficient every 4 extend
void SlerpSE3Curve::setSamplingRatio(const int ratio) {
  slerpPolicy_.setMinimumMeasurements(ratio);
}

void SlerpSE3Curve::clear() {
  manager_.clear();
}

}  // namespace curves
