/*
 * CubicHermiteE3Curve.hpp
 *
 *  Created on: Aug, 2016
 *      Author: Christian Gehring
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <kindr/Core>

#include "curves/LocalSupport2CoefficientManager.hpp"
#include "curves/SE3Curve.hpp"

struct HermiteE3Knot {
  using Position = Eigen::Vector3d;
  using Velocity = Eigen::Vector3d;
  using Acceleration = Eigen::Vector3d;

 public:
  HermiteE3Knot(Position position, Velocity velocity) : position_(std::move(position)), velocity_(std::move(velocity)) {}
  HermiteE3Knot() : HermiteE3Knot(Position::Zero(), Velocity::Zero()) {}
  HermiteE3Knot(const HermiteE3Knot&) = default;
  HermiteE3Knot& operator=(const HermiteE3Knot&) = default;
  HermiteE3Knot(HermiteE3Knot&&) = default;
  HermiteE3Knot& operator=(HermiteE3Knot&&) = default;
  virtual ~HermiteE3Knot() = default;

  Position getPosition() const { return position_; }

  Velocity getVelocity() const { return velocity_; }

  void setPosition(const Position& position) { position_ = position; }

  void setVelocity(const Velocity& velocity) { velocity_ = velocity; }

 private:
  Position position_;
  Velocity velocity_;
};

namespace curves {

/// Implements the Cubic Hermite curve class. See KimKimShin paper.
/// The Hermite interpolation function is defined, with the respective Jacobians regarding  A and B:
//
/// Translations:
/// Equations for the unit interval:
// Let t_W_A, t_W_B denote the control point values (=translations) and W_v_W_A, W_v_W_B
// the control derivatives (=velocities in R³) at the interval's boundaries.
// Then (b == beta):
// p0 = t_W_A, p1 = t_W_B, p2 = W_v_W_A, p3 = W_v_W_B
// b0 = 2t³-3t²+1, b_1 = -2t³+3t², b_2 = t³-2t²+t, b_3 = t³-t²
// Spline equation:
// p(t) = p0 * b0 + p1 * b1 + p2 * b2 + p3 + b3

class CubicHermiteE3Curve {
 public:
  using Coefficient = HermiteE3Knot;
  using CoefficientIter = LocalSupport2CoefficientManager<Coefficient>::CoefficientIter;
  using ValueType = HermiteE3Knot::Position;
  using DerivativeType = HermiteE3Knot::Velocity;
  using Acceleration = HermiteE3Knot::Acceleration;

 public:
  /// Print the value of the coefficient, for debugging and unit tests
  virtual void print(const std::string& str) const;

  virtual bool writeEvalToFile(const std::string& filename, int nSamples) const;

  /// The first valid time for the curve.
  virtual Time getMinTime() const;

  /// The one past the last valid time for the curve.
  virtual Time getMaxTime() const;

  bool isEmpty() const;

  // return number of coefficients curve is composed of
  int size() const;

  /// \brief calculate the slope between 2 coefficients
  static DerivativeType calculateSlope(const Time& timeA, const Time& timeB, const ValueType& coeffA, const ValueType& coeffB) {
    const double inverse_dt_sec = 1.0 / double(timeB - timeA);
    // Original curves implementation was buggy for 180 deg flips.

    // Calculate the global angular velocity:
    const DerivativeType velocity_m_s = (coeffB - coeffA) * inverse_dt_sec;
    // note: unit of derivative is m/s for first 3 and rad/s for last 3 entries
    return velocity_m_s;
  }

  /// Extend the curve so that it can be evaluated at these times.
  /// Try to make the curve fit to the values.
  /// Note: Assumes that extend times strictly increase the curve time
  virtual void extend(const std::vector<Time>& times, const std::vector<ValueType>& values, std::vector<Key>* outKeys);

  /// \brief Fit a new curve to these data points.
  ///
  /// The existing curve will be cleared.fitCurveWithDerivatives
  /// Underneath the curve should have some default policy for fitting.
  virtual void fitCurve(const std::vector<Time>& times, const std::vector<ValueType>& values, std::vector<Key>* outKeys);

  virtual void fitCurveWithDerivatives(const std::vector<Time>& times, const std::vector<ValueType>& values,
                                       const DerivativeType& initialDerivative, const DerivativeType& finalDerivative,
                                       std::vector<Key>* outKeys);

  virtual void fitPeriodicCurve(const std::vector<Time>& times, const std::vector<ValueType>& values, std::vector<Key>* outKeys);
  /// Evaluate the ambient space of the curve.
  virtual bool evaluate(ValueType& value, Time time) const;

  /// Evaluate the curve derivatives.
  virtual bool evaluateDerivative(DerivativeType& derivative, Time time, unsigned int derivativeOrder) const;

  bool evaluateLinearAcceleration(Acceleration& linearAcceleration, Time time) const;

  // clear the curve
  virtual void clear();

 private:
  LocalSupport2CoefficientManager<Coefficient> manager_;
};

}  // namespace curves
