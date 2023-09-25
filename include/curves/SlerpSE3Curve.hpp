#pragma once

#include <kindr/Core>

#include "curves/CubicHermiteSE3Curve.hpp"
#include "curves/LocalSupport2CoefficientManager.hpp"
#include "curves/SE3CompositionCurve.hpp"
#include "curves/SE3Curve.hpp"
#include "curves/SamplingPolicy.hpp"

namespace curves {

/// Implements the Slerp (Spherical linear interpolation) curve class.
/// The Slerp interpolation function is defined as, with the respective Jacobians regarding  A and B:
/// \f[ T = A(A^{-1}B)^{\alpha} \f]
class SlerpSE3Curve : public SE3Curve {
  friend class SE3CompositionCurve<SlerpSE3Curve, SlerpSE3Curve>;
  friend class SE3CompositionCurve<SlerpSE3Curve, CubicHermiteSE3Curve>;
  friend class SamplingPolicy;

 public:
  using ValueType = SE3Curve::ValueType;
  using DerivativeType = SE3Curve::DerivativeType;
  using Coefficient = ValueType;
  using CoefficientIter = LocalSupport2CoefficientManager<Coefficient>::CoefficientIter;

  /// The first valid time for the curve.
  Time getMinTime() const override;

  /// The one past the last valid time for the curve.
  Time getMaxTime() const override;

  bool isEmpty() const override;

  /// return number of coefficients curve is composed of
  int size() const override;

  /// Extend the curve so that it can be evaluated at these times.
  /// Try to make the curve fit to the values.
  /// Underneath the curve should have some default policy for fitting.
  void extend(const std::vector<Time>& times, const std::vector<ValueType>& values, std::vector<Key>* outKeys) override;

  /// \brief Fit a new curve to these data points.
  ///
  /// The existing curve will be cleared.
  /// Underneath the curve should have some default policy for fitting.
  void fitCurve(const std::vector<Time>& times, const std::vector<ValueType>& values, std::vector<Key>* outKeys) override;

  /// set minimum sampling period
  void setMinSamplingPeriod(Time time) override;

  /// \brief Set the sampling ratio.
  ///   eg. 4 will add a coefficient every 4 extend
  void setSamplingRatio(int ratio) override;

  void clear() override;

 private:
  LocalSupport2CoefficientManager<Coefficient> manager_;
  SamplingPolicy slerpPolicy_;
};

using SE3 = kindr::HomogeneousTransformationPosition3RotationQuaternionD;
using SO3 = SE3::Rotation;

// extend policy for slerp curves
template <>
inline void SamplingPolicy::extend<SlerpSE3Curve, SE3>(const std::vector<Time>& times, const std::vector<SE3>& values, SlerpSE3Curve* curve,
                                                       std::vector<Key>* outKeys) {
  // todo: deal with minSamplingPeriod_ when extending with multiple times
  if (times.size() != 1) {
    curve->manager_.insertCoefficients(times, values, outKeys);
  } else {
    // If the curve is empty or of size 1, simply add the new coefficient
    if (curve->isEmpty() || curve->size() == 1) {
      curve->manager_.insertCoefficients(times, values, outKeys);
    } else {
      if (minimumMeasurements_ == 1) {
        curve->manager_.addCoefficientAtEnd(times[0], values[0], outKeys);
      } else {
        ++measurementsSinceLastExtend_;

        if (measurementsSinceLastExtend_ == 1) {
          curve->manager_.addCoefficientAtEnd(times[0], values[0], outKeys);
        } else {
          auto itPrev = (--curve->manager_.coefficientEnd());
          curve->manager_.modifyCoefficient(itPrev, times[0], values[0]);
        }
        if (measurementsSinceLastExtend_ == minimumMeasurements_) {
          measurementsSinceLastExtend_ = 0;
        }
      }
    }
  }
}

}  // namespace curves
