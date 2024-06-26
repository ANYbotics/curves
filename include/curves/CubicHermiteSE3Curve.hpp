#pragma once

#include <utility>

#include <kindr/Core>

#include "curves/LocalSupport2CoefficientManager.hpp"
#include "curves/SE3CompositionCurve.hpp"
#include "curves/SE3Curve.hpp"
#include "curves/SamplingPolicy.hpp"

namespace kindr {

/**
 * Wrapper class for Hermite-style coefficients (made of QuatTransformation and Vector6)
 *
 * @tparam Scalar The number type of the coefficients.
 */
template <typename Scalar>
struct HermiteTransformation {
  using Transform = kindr::HomTransformQuatD;
  using Twist = kindr::TwistGlobalD;

 public:
  HermiteTransformation(Transform transform, Twist derivatives)
      : transformation_(std::move(transform)), transformationDerivative_(std::move(derivatives)){};

  Transform getTransformation() const { return transformation_; }

  Twist getTransformationDerivative() const { return transformationDerivative_; }

 private:
  Transform transformation_;
  Twist transformationDerivative_;
};

}  // namespace kindr

namespace curves {

using ValueType = SE3Curve::ValueType;
using DerivativeType = SE3Curve::DerivativeType;
using Coefficient = kindr::HermiteTransformation<double>;
using TimeToKeyCoefficientMap = LocalSupport2CoefficientManager<Coefficient>::TimeToKeyCoefficientMap;
using CoefficientIter = LocalSupport2CoefficientManager<Coefficient>::CoefficientIter;

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
//
/// Rotations:
/// Equations for the unit interval:
// Let quat_W_A, quat_W_B denote the control point values (=unit quaternions) and va, vb
// the control derivatives (=angular speeds in R³) at the interval's boundaries.
// Then (w == omega, b == beta):
// w_1 = va / 3
// w_2 = log[ exp(w_1)^{-1} * quat_W_A^{-1} * quat_W_B * exp(w_3)^{-1} ]
// w_3 = vb / 3
// b_1 = t³-3t²+3t, b_2 = -2t³+3t², b_3 = t³
// Spline equation:
// q(t) = p_1 * exp(w_1*b_1) * exp(w_2*b_2) * exp(w_3*b_3)

class CubicHermiteSE3Curve : public SE3Curve {
  friend class SamplingPolicy;

 public:
  using Coefficient = kindr::HermiteTransformation<double>;

  CubicHermiteSE3Curve();

  /// The first valid time for the curve.
  Time getMinTime() const override;

  /// The one past the last valid time for the curve.
  Time getMaxTime() const override;

  bool isEmpty() const override;

  // return number of coefficients curve is composed of
  int size() const override;

  /// \brief calculate the slope between 2 coefficients
  static DerivativeType calculateSlope(const Time& timeA, const Time& timeB, const ValueType& coeffA, const ValueType& coeffB);

  /// Extend the curve so that it can be evaluated at these times.
  /// Try to make the curve fit to the values.
  /// Note: Assumes that extend times strictly increase the curve time
  void extend(const std::vector<Time>& times, const std::vector<ValueType>& values, std::vector<Key>* outKeys) override;

  /// \brief Fit a new curve to these data points.
  ///
  /// The existing curve will be cleared.fitCurveWithDerivatives
  /// Underneath the curve should have some default policy for fitting.
  void fitCurve(const std::vector<Time>& times, const std::vector<ValueType>& values, std::vector<Key>* outKeys) override;

  void fitCurveWithDerivatives(const std::vector<Time>& times, const std::vector<ValueType>& values,
                               const DerivativeType& initialDerivative = DerivativeType(),
                               const DerivativeType& finalDerivative = DerivativeType(), std::vector<Key>* outKeys = nullptr);

  /// Evaluate the ambient space of the curve.
  bool evaluate(ValueType& value, Time time) const override;

  /// Evaluate the curve derivatives.
  bool evaluateDerivative(DerivativeType& derivative, Time time, unsigned int derivativeOrder) const override;

  bool evaluateLinearAcceleration(kindr::Acceleration3D& linearAcceleration, Time time);

  // set the minimum sampling period
  void setMinSamplingPeriod(Time time) override;

  /// \brief Set the sampling ratio.
  ///   eg. 4 will add a coefficient every 4 extend
  void setSamplingRatio(int ratio) override;

  // clear the curve
  void clear() override;

 private:
  LocalSupport2CoefficientManager<Coefficient> manager_;
  SamplingPolicy hermitePolicy_;
};

using SE3 = kindr::HomogeneousTransformationPosition3RotationQuaternionD;
using RotationQuaternion = kindr::RotationQuaternionPD;
using RotationQuaternionDiff = kindr::RotationQuaternionDiffD;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Twist = kindr::TwistGlobalD;

// implements the special (extend) policies for Cubic Hermite curves
template <>
inline Key SamplingPolicy::defaultExtend<CubicHermiteSE3Curve, ValueType>(const Time& time, const ValueType& value,
                                                                          CubicHermiteSE3Curve* curve) {
  DerivativeType derivative;
  // cases:

  if (curve->manager_.empty()) {
    // manager is empty
    derivative.setZero();
  } else if (curve->manager_.size() == 1) {
    // 1 value in manager (2 in total)
    // get latest coefficient from manager
    CoefficientIter last = curve->manager_.coefficientBegin();
    // calculate slope
    // note: unit of derivative is m/s for first 3 and rad/s for last 3 entries
    derivative = CubicHermiteSE3Curve::calculateSlope(last->first, time, last->second.coefficient_.getTransformation(), value);
    // update previous coefficient
    Coefficient updated(last->second.coefficient_.getTransformation(), derivative);
    curve->manager_.updateCoefficientByKey(last->second.key_, updated);
  } else if (curve->manager_.size() > 1) {
    // more than 1 values in manager
    // get latest 2 coefficients from manager
    CoefficientIter rVal0;
    CoefficientIter rVal1;
    CoefficientIter last = --curve->manager_.coefficientEnd();
    curve->manager_.getCoefficientsAt(last->first, &rVal0, &rVal1);

    // update derivative of previous coefficient
    DerivativeType derivative0;
    derivative0 = CubicHermiteSE3Curve::calculateSlope(rVal0->first, time, rVal0->second.coefficient_.getTransformation(), value);
    Coefficient updated(rVal1->second.coefficient_.getTransformation(), derivative0);
    curve->manager_.updateCoefficientByKey(rVal1->second.key_, updated);

    // calculate slope
    derivative = CubicHermiteSE3Curve::calculateSlope(rVal1->first, time, rVal1->second.coefficient_.getTransformation(), value);
  }
  measurementsSinceLastExtend_ = 0;
  lastExtend_ = time;
  return curve->manager_.insertCoefficient(time, Coefficient(value, derivative));
}

template <>
inline Key SamplingPolicy::interpolationExtend<CubicHermiteSE3Curve, ValueType>(const Time& time, const ValueType& value,
                                                                                CubicHermiteSE3Curve* curve) {
  DerivativeType derivative;
  if (measurementsSinceLastExtend_ == 0) {
    // extend curve with new interpolation coefficient if necessary
    --curve->manager_.coefficientEnd();
    CoefficientIter last = --curve->manager_.coefficientEnd();
    derivative = last->second.coefficient_.getTransformationDerivative();
  } else {
    // assumes the interpolation coefficient is already set (at end of curve)
    // assumes same velocities as last Coefficient
    CoefficientIter rVal0;
    CoefficientIter rValInterp;
    CoefficientIter last = --curve->manager_.coefficientEnd();
    curve->manager_.getCoefficientsAt(last->first, &rVal0, &rValInterp);

    derivative = CubicHermiteSE3Curve::calculateSlope(rVal0->first, time, rVal0->second.coefficient_.getTransformation(), value);

    // update the interpolated coefficient with given values and velocities from last coefficeint
    curve->manager_.removeCoefficientAtTime(rValInterp->first);
  }

  ++measurementsSinceLastExtend_;
  return curve->manager_.insertCoefficient(time, Coefficient(value, derivative));
}

template <>
inline void SamplingPolicy::extend<CubicHermiteSE3Curve, ValueType>(const std::vector<Time>& times, const std::vector<ValueType>& values,
                                                                    CubicHermiteSE3Curve* curve, std::vector<Key>* /*outKeys*/) {
  for (std::size_t i = 0; i < times.size(); ++i) {
    // ensure time strictly increases
    assert(((times[i] > curve->manager_.getMaxTime()) || curve->manager_.empty()) && "Curve can only be extended into the future.");
    if (curve->manager_.empty()) {
      defaultExtend(times[i], values[i], curve);
    } else if ((measurementsSinceLastExtend_ >= minimumMeasurements_ && lastExtend_ + minSamplingPeriod_ < times[i])) {
      // delete interpolated coefficient
      CoefficientIter last = --curve->manager_.coefficientEnd();
      curve->manager_.removeCoefficientAtTime(last->first);
      // todo write outkeys
      defaultExtend(times[i], values[i], curve);
    } else {
      interpolationExtend(times[i], values[i], curve);
    }
  }
}

}  // namespace curves

namespace kindr {

using ValueType = curves::SE3Curve::ValueType;
using DerivativeType = curves::SE3Curve::DerivativeType;

}  // namespace kindr
