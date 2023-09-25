#pragma once

#include "curves/SE3Curve.hpp"

namespace curves {

// SE3CompositionCurve is a curve composed of a base and a correction curve.
// The corrections can be sampled at a lower frequency than the base curve,
// therefore reducing the optimization state space. The corrections are applied
// on the left side.

template <class C1, class C2>
class SE3CompositionCurve : public SE3Curve {
 private:
  C1 baseCurve_;
  C2 correctionCurve_;

 public:
  using ValueType = SE3Curve::ValueType;
  using DerivativeType = SE3Curve::DerivativeType;

  /// \brief Returns the first valid time for the curve.
  Time getMinTime() const override;

  /// \brief Returns the last valid time for the curve.
  Time getMaxTime() const override;

  /// \brief Checks if the curve is empty.
  bool isEmpty() const override;

  /// \brief Returns the number of coefficients in the correction curve
  int size() const override;

  /// \brief Extend the curve so that it can be evaluated at these times by
  ///        using a default correction sampling policy.
  void extend(const std::vector<Time>& times, const std::vector<ValueType>& values, std::vector<Key>* outKeys) override;

  /// \brief Set the minimum sampling period for the correction curve.
  ///        Overloads the function defined in SE3Curve base class.
  void setMinSamplingPeriod(Time minSamplingPeriod) override;

  /// \brief Set the sampling ratio for the correction curve.
  ///   eg. 4 will add a coefficient every 4 extend
  void setSamplingRatio(int ratio) override;

  /// \brief Fit a new curve to these data points.
  ///
  /// The existing curve will be cleared.
  /// Underneath the curve should have some default policy for fitting.
  void fitCurve(const std::vector<Time>& times, const std::vector<ValueType>& values, std::vector<Key>* outKeys) override;

  /// Evaluate the ambient space of the curve.
  virtual ValueType evaluate(Time time) const;

  /// Evaluate the curve derivatives.
  /// linear 1st derivative has following behaviour:
  /// - time is out of bound --> error
  /// - time is between 2 coefficients --> take slope between the 2 coefficients
  /// - time is on coefficient (not last coefficient) --> take slope between coefficient and next coefficients
  /// - time is on last coefficient --> take slope between last-1 and last coefficient
  /// derivatives of order >1 equal 0
  bool evaluateDerivative(DerivativeType& derivative, Time time, unsigned derivativeOrder) const override;

  /// \brief Clear the base and correction curves.
  void clear() override;
};

}  // namespace curves

#include "SE3CompositionCurve.tpp"
