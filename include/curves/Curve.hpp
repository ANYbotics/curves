#pragma once

#include <string>
#include <vector>

namespace curves {

using Time = double;
using Key = std::size_t;

template <typename CurveConfig>
class Curve {
 public:
  /// The value type of the curve.
  using ValueType = typename CurveConfig::ValueType;

  /// The curve's derivative type.
  using DerivativeType = typename CurveConfig::DerivativeType;

  Curve() = default;
  Curve(const Curve&) = default;
  Curve(Curve&&) = default;

  virtual ~Curve() = default;

  Curve& operator=(const Curve&) = default;
  Curve& operator=(Curve&&) = default;

  ///\defgroup Info
  ///\name Methods to get information about the curve.
  ///@{

  /// The first valid time of the curve.
  virtual Time getMinTime() const = 0;

  /// The one past the last valid time for the curve.
  virtual Time getMaxTime() const = 0;
  ///@}

  /// \name Methods to evaluate the curve
  ///@{

  /// Evaluate the ambient space of the curve.
  virtual bool evaluate(ValueType& value, Time time) const = 0;

  //  /// Evaluate the curve derivatives.
  virtual bool evaluateDerivative(DerivativeType& derivative, Time time, unsigned derivativeOrder) const = 0;

  ///@}

  /// \name Methods to fit the curve based on data.
  ///@{

  /// Extend the curve so that it can be evaluated at these times.
  /// Try to make the curve fit to the values.
  /// Underneath the curve should have some default policy for fitting.
  virtual void extend(const std::vector<Time>& times, const std::vector<ValueType>& values, std::vector<Key>* outKeys) = 0;

  /// \brief Fit a new curve to these data points.
  ///
  /// The existing curve will be cleared.
  /// Underneath the curve should have some default policy for fitting.
  virtual void fitCurve(const std::vector<Time>& times, const std::vector<ValueType>& values, std::vector<Key>* outKeys) = 0;

  ///@}

  /// \brief Clear all the curve coefficients
  virtual void clear() = 0;
};

}  // namespace curves
