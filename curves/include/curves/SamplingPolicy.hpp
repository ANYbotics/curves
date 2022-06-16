/*
 * @file SamplingPolicy.hpp
 * @date Mar 02, 2015
 * @author Abel Gawel
 */

#ifndef SAMPLING_POLICY_HPP
#define SAMPLING_POLICY_HPP

#include <typeinfo>

namespace curves {

class SamplingPolicy {
 private:
  int measurementsSinceLastExtend_;
  int minimumMeasurements_;
  Time minSamplingPeriod_;
  Time lastExtend_;

 public:
  SamplingPolicy() : measurementsSinceLastExtend_(0), minimumMeasurements_(1), minSamplingPeriod_(0), lastExtend_(0) {}

  SamplingPolicy(int minimumMeasurements, Time minSamplingPeriod)
      : measurementsSinceLastExtend_(0), minimumMeasurements_(minimumMeasurements), minSamplingPeriod_(minSamplingPeriod), lastExtend_(0) {}

  template <typename CurveType, typename ValueType>
  Key interpolationExtend(const Time& time, const ValueType& value, CurveType* curve);

  template <typename CurveType, typename ValueType>
  Key defaultExtend(const Time& time, const ValueType& value, CurveType* curve);

  template <typename CurveType, typename ValueType>
  void extend(const std::vector<Time>& times, const std::vector<ValueType>& values, CurveType* curve, std::vector<Key>* outKeys);

  /// Print the value of the coefficient, for debugging and unit tests
  int getMeasurementsSinceLastExtend() const { return measurementsSinceLastExtend_; }

  int getMinimumMeasurements() const { return minimumMeasurements_; }

  Time getMinSamplingPeriod() const { return minSamplingPeriod_; }

  Time getLastExtendTime() const { return lastExtend_; }

  void setLastExtendTime(Time time) { lastExtend_ = time; }

  void setMinimumMeasurements(int n) { minimumMeasurements_ = n; }

  void setMinSamplingPeriod(Time minSamplingPeriod) { minSamplingPeriod_ = minSamplingPeriod; }

  void incrementMeasurementsTaken(int num) { measurementsSinceLastExtend_ = measurementsSinceLastExtend_ + num; }

  void setMeasurementsSinceLastExtend(int num) { measurementsSinceLastExtend_ = num; }
};

}  // namespace curves

#endif /* SAMPLING_POLICY_HPP */
