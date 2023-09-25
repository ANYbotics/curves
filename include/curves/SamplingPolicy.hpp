#pragma once

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

  template <typename CurveType, typename ValueType>
  Key interpolationExtend(const Time& time, const ValueType& value, CurveType* curve);

  template <typename CurveType, typename ValueType>
  Key defaultExtend(const Time& time, const ValueType& value, CurveType* curve);

  template <typename CurveType, typename ValueType>
  void extend(const std::vector<Time>& times, const std::vector<ValueType>& values, CurveType* curve, std::vector<Key>* outKeys);

  void setMinimumMeasurements(int n) { minimumMeasurements_ = n; }

  void setMinSamplingPeriod(Time minSamplingPeriod) { minSamplingPeriod_ = minSamplingPeriod; }
};

}  // namespace curves
