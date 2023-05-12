/*
 * SlerpSE3Curve.hpp
 *
 *  Created on: Oct 10, 2014
 *      Author: Renaud Dube, Abel Gawel, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "CubicHermiteSE3Curve.hpp"
#include "LocalSupport2CoefficientManager.hpp"
#include "SE3CompositionCurve.hpp"
#include "SE3Curve.hpp"
#include "SamplingPolicy.hpp"
#include "kindr/Core"

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
  using TimeToKeyCoefficientMap = LocalSupport2CoefficientManager<Coefficient>::TimeToKeyCoefficientMap;
  using CoefficientIter = LocalSupport2CoefficientManager<Coefficient>::CoefficientIter;

  /// Print the value of the coefficient, for debugging and unit tests
  void print(const std::string& str) const override;

  /// The first valid time for the curve.
  Time getMinTime() const override;

  /// The one past the last valid time for the curve.
  Time getMaxTime() const override;

  bool isEmpty() const override;

  // return number of coefficients curve is composed of
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

  /// \brief Set some coefficients of the curve
  /// The existing curve will NOT be cleared.
  void setCurve(const std::vector<Time>& times, const std::vector<ValueType>& values);

  /// Evaluate the ambient space of the curve.
  virtual ValueType evaluate(Time time) const;

  /// Evaluate the curve derivatives.
  /// linear 1st derivative has following behaviour:
  /// - time is out of bound --> error
  /// - time is between 2 coefficients --> take slope between the 2 coefficients
  /// - time is on coefficient (not last coefficient) --> take slope between coefficient and next coefficients
  /// - time is on last coefficient --> take slope between last-1 and last coefficient
  /// derivatives of order >1 equal 0
  virtual DerivativeType evaluateDerivative(Time time, unsigned derivativeOrder) const;

  virtual void setTimeRange(Time minTime, Time maxTime);

  /// \brief Evaluate the angular velocity of Frame b as seen from Frame a, expressed in Frame a.
  Eigen::Vector3d evaluateAngularVelocityA(Time time) override;

  /// \brief Evaluate the angular velocity of Frame a as seen from Frame b, expressed in Frame b.
  Eigen::Vector3d evaluateAngularVelocityB(Time time) override;

  /// \brief Evaluate the velocity of Frame b as seen from Frame a, expressed in Frame a.
  Eigen::Vector3d evaluateLinearVelocityA(Time time) override;

  /// \brief Evaluate the velocity of Frame a as seen from Frame b, expressed in Frame b.
  Eigen::Vector3d evaluateLinearVelocityB(Time time) override;

  /// \brief evaluate the velocity/angular velocity of Frame b as seen from Frame a,
  ///        expressed in Frame a. The return value has the linear velocity (0,1,2),
  ///        and the angular velocity (3,4,5).
  Vector6d evaluateTwistA(Time time) override;

  /// \brief evaluate the velocity/angular velocity of Frame a as seen from Frame b,
  ///        expressed in Frame b. The return value has the linear velocity (0,1,2),
  ///        and the angular velocity (3,4,5).
  Vector6d evaluateTwistB(Time time) override;

  /// \brief Evaluate the angular derivative of Frame b as seen from Frame a, expressed in Frame a.
  Eigen::Vector3d evaluateAngularDerivativeA(unsigned derivativeOrder, Time time) override;

  /// \brief Evaluate the angular derivative of Frame a as seen from Frame b, expressed in Frame b.
  Eigen::Vector3d evaluateAngularDerivativeB(unsigned derivativeOrder, Time time) override;

  /// \brief Evaluate the derivative of Frame b as seen from Frame a, expressed in Frame a.
  Eigen::Vector3d evaluateLinearDerivativeA(unsigned derivativeOrder, Time time) override;

  /// \brief Evaluate the derivative of Frame a as seen from Frame b, expressed in Frame b.
  Eigen::Vector3d evaluateLinearDerivativeB(unsigned derivativeOrder, Time time) override;

  /// \brief evaluate the velocity/angular derivative of Frame b as seen from Frame a,
  ///        expressed in Frame a. The return value has the linear velocity (0,1,2),
  ///        and the angular velocity (3,4,5).
  Vector6d evaluateDerivativeA(unsigned derivativeOrder, Time time) override;

  /// \brief evaluate the velocity/angular velocity of Frame a as seen from Frame b,
  ///        expressed in Frame b. The return value has the linear velocity (0,1,2),
  ///        and the angular velocity (3,4,5).
  Vector6d evaluateDerivativeB(unsigned derivativeOrder, Time time) override;

  // set minimum sampling period
  void setMinSamplingPeriod(Time time) override;

  /// \brief Set the sampling ratio.
  ///   eg. 4 will add a coefficient every 4 extend
  void setSamplingRatio(int ratio) override;

  void clear() override;

  /// \brief Perform a rigid transformation on the left side of the curve
  void transformCurve(ValueType T) override;

  void saveCurveTimesAndValues(const std::string& filename) const override;

  void saveCurveAtTimes(const std::string& filename, std::vector<Time> times) const override;

  void saveCorrectionCurveAtTimes(const std::string& /*filename*/, std::vector<Time> /*times*/) const override {}

  void getCurveTimes(std::vector<Time>* outTimes) const override;

  // Fake functions to comply with the current interfaces of trajectories_optimizer
  // todo : tidy up

  /// \brief Returns the number of coefficients in the correction curve
  int correctionSize() const override { return 0; }

  /// \brief Fold in the correction curve into the base curve and reinitialize
  ///        correction curve coefficients to identity transformations.
  void foldInCorrections() override {}

  /// \brief Add coefficients to the correction curve at given times.
  void setCorrectionTimes(const std::vector<Time>& /*times*/) override {}

  /// \brief Remove a correction coefficient at the specified time.
  void removeCorrectionCoefficientAtTime(Time /*time*/) override {}

  /// \brief Set the correction coefficient value at the specified time.
  void setCorrectionCoefficientAtTime(Time /*time*/, ValueType /*value*/) override {}

  /// \brief Reset the correction curve to identity values with knots at desired times
  void resetCorrectionCurve(const std::vector<Time>& /*times*/) override {}

  /// \brief Set the base curve to given values with knots at desired times
  /// Resets the curve beforehand.
  void setBaseCurve(const std::vector<Time>& /*times*/, const std::vector<ValueType>& /*values*/) override {}

  /// \brief Add / replace the given coefficients without resetting the curve.
  void setBaseCurvePart(const std::vector<Time>& /*times*/, const std::vector<ValueType>& /*values*/) override {}

  /// \brief Modifies values of the base coefficient in batch, starting at times[0] and assuming that
  /// a coefficient exists at all the specified times.
  void modifyBaseCoefficientsValuesInBatch(const std::vector<Time>& /*times*/, const std::vector<ValueType>& /*values*/) override {}

  void getBaseCurveTimes(std::vector<Time>* /*outTimes*/) const override {}

  void getBaseCurveTimesInWindow(std::vector<Time>* /*outTimes*/, Time /*begTime*/, Time /*endTime*/) const override {}

  // return number of coefficients curve is composed of
  int baseSize() const override { return size(); }

  void saveCorrectionCurveTimesAndValues(const std::string& /*filename*/) const override {}

 private:
  LocalSupport2CoefficientManager<Coefficient> manager_;
  SamplingPolicy slerpPolicy_;
};

using SE3 = kindr::HomogeneousTransformationPosition3RotationQuaternionD;
using SO3 = SE3::Rotation;
using AngleAxis = kindr::AngleAxisPD;

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
