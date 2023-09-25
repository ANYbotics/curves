#include "curves/SE3CompositionCurve.hpp"

namespace curves {

template <class C1, class C2>
Time SE3CompositionCurve<C1, C2>::getMinTime() const {
  return baseCurve_.getMinTime();
}

template <class C1, class C2>
Time SE3CompositionCurve<C1, C2>::getMaxTime() const {
  return baseCurve_.getMaxTime();
}

template <class C1, class C2>
bool SE3CompositionCurve<C1, C2>::isEmpty() const {
  return baseCurve_.isEmpty();
}

template <class C1, class C2>
int SE3CompositionCurve<C1, C2>::size() const {
  //  return baseCurve_.size();
  return correctionCurve_.size();
}

template <class C1, class C2>
void SE3CompositionCurve<C1, C2>::setMinSamplingPeriod(const Time minSamplingPeriod) {
  baseCurve_.setMinSamplingPeriod(0);
  correctionCurve_.setMinSamplingPeriod(minSamplingPeriod);
}

template <class C1, class C2>
void SE3CompositionCurve<C1, C2>::setSamplingRatio(const int ratio) {
  baseCurve_.setSamplingRatio(1);
  correctionCurve_.setSamplingRatio(ratio);
}

template <class C1, class C2>
void SE3CompositionCurve<C1, C2>::extend(const std::vector<Time>& times,
                                         const std::vector<typename SE3CompositionCurve<C1, C2>::ValueType>& values,
                                         std::vector<Key>* outKeys) {
  // todo: Treat case when times.size() != 1
  assert(times.size() == 1 && "Extend was called with more than one time.");
  assert(values.size() == 1 && "Extend was called with more than one value.");

  // Find the new limit times of the curve
  Time newMaxTime = 0;
  Time newMinTime = 0;

  if (!baseCurve_.isEmpty()) {
    newMaxTime = baseCurve_.getMaxTime();
    newMinTime = baseCurve_.getMinTime();
  }

  for (auto time : times) {
    if (newMaxTime < time) {
      newMaxTime = time;
    }
    if (newMinTime > time) {
      newMinTime = time;
    }
  }

  // Extend the correction curves to these times
  std::vector<Time> correctionTimes;
  std::vector<ValueType> correctionValues;

  if (correctionCurve_.isEmpty()) {
    correctionTimes.push_back(newMinTime);
    correctionValues.emplace_back(ValueType::Position(0, 0, 0), ValueType::Rotation(1, 0, 0, 0));
    correctionCurve_.extend(correctionTimes, correctionValues);
  }

  if (correctionCurve_.getMaxTime() < newMaxTime) {
    correctionTimes.push_back(newMaxTime);
    correctionValues.push_back(correctionCurve_.evaluate(correctionCurve_.getMaxTime()));
    correctionCurve_.extend(correctionTimes, correctionValues);
  }

  if (correctionCurve_.getMinTime() > newMinTime) {
    correctionTimes.push_back(newMinTime);
    correctionValues.push_back(correctionCurve_.evaluate(correctionCurve_.getMinTime()));
    correctionCurve_.extend(correctionTimes, correctionValues);
  }

  // Compute the base curve updates accounting for the corrections
  std::vector<ValueType> newValues;
  auto itValues = values.begin();
  for (auto time : times) {
    newValues.push_back(correctionCurve_.evaluate(time).inverted() * (*itValues));
    ++itValues;
  }
  baseCurve_.extend(times, newValues, outKeys);
}

template <class C1, class C2>
void SE3CompositionCurve<C1, C2>::fitCurve(const std::vector<Time>& times,
                                           const std::vector<typename SE3CompositionCurve<C1, C2>::ValueType>& values,
                                           std::vector<Key>* outKeys) {
  extend(times, values, outKeys);
}

template <class C1, class C2>
typename SE3CompositionCurve<C1, C2>::ValueType SE3CompositionCurve<C1, C2>::evaluate(Time time) const {
  return correctionCurve_.evaluate(time) * baseCurve_.evaluate(time);
}

template <class C1, class C2>
void SE3CompositionCurve<C1, C2>::clear() {
  baseCurve_.clear();
  correctionCurve_.clear();
}

}  // namespace curves
