/*
 * PolynomialSplineScalarCurve.hpp
 *
 *  Created on: Mar 5, 2015
 *      Author: Péter Fankhauser, Dario Bellicoso
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// stl
#include <string>
#include <vector>

// glog
#include <glog/logging.h>

// curves
#include "curves/Curve.hpp"
#include "curves/PolynomialSplineContainer.hpp"
#include "curves/ScalarCurveConfig.hpp"
#include "curves/polynomial_splines_containers.hpp"

namespace curves {

template <typename SplineContainerType_>
class PolynomialSplineScalarCurve : public Curve<ScalarCurveConfig> {
 public:
  using Parent = Curve<ScalarCurveConfig>;
  using ValueType = typename Parent::ValueType;
  using DerivativeType = typename Parent::DerivativeType;

  using SplineContainerType = SplineContainerType_;

  PolynomialSplineScalarCurve() : Parent(), container_(), minTime_(0.0) {}

  void print(const std::string& /*str*/) const override {
    const double minTime = getMinTime();
    const double maxTime = getMaxTime();
    double timeAtEval = minTime;
    int nPoints = 15;
    double timeDiff = (maxTime - minTime) / (nPoints - 1);

    for (int i = 0; i < nPoints; i++) {
      double firstDerivative = 0.;
      double secondDerivative = 0.;
      double value = 0.;
      evaluate(value, timeAtEval);
      evaluateDerivative(firstDerivative, timeAtEval, 1);
      evaluateDerivative(secondDerivative, timeAtEval, 2);
      printf("t: %lf, x: %lf dx: %lf dxx: %lf\n", timeAtEval, value, firstDerivative, secondDerivative);
      timeAtEval += timeDiff;
    }
  }

  Time getMinTime() const override { return minTime_; }

  Time getMaxTime() const override { return container_.getContainerDuration() + minTime_; }

  bool evaluate(ValueType& value, Time time) const override {
    time -= minTime_;
    value = container_.getPositionAtTime(time);
    return true;
  }

  bool evaluateDerivative(DerivativeType& value, Time time, unsigned derivativeOrder) const override {
    time -= minTime_;
    switch (derivativeOrder) {
      case (1): {
        value = container_.getVelocityAtTime(time);
      } break;

      case (2): {
        value = container_.getAccelerationAtTime(time);
      } break;

      default:
        return false;
    }

    return true;
  }

  void extend(const std::vector<Time>& /*times*/, const std::vector<ValueType>& /*values*/, std::vector<Key>* /*outKeys*/) override {
    throw std::runtime_error("extend is not yet implemented!");
  }

  void fitCurve(const std::vector<Time>& times, const std::vector<ValueType>& values, std::vector<Key>* /*outKeys*/) override {
    container_.setData(times, values, 0.0, 0.0, 0.0, 0.0);
    minTime_ = times.front();
  }

  virtual void fitCurve(const std::vector<Time>& times, const std::vector<ValueType>& values, double initialVelocity,
                        double initialAcceleration, double finalVelocity, double finalAcceleration, std::vector<Key>* /*outKeys*/) {
    container_.setData(times, values, initialVelocity, initialAcceleration, finalVelocity, finalAcceleration);
    minTime_ = times.front();
  }

  virtual void fitCurve(const std::vector<SplineOptions>& optionList, std::vector<Key>* /*outKeys*/) {
    if (optionList.size() > container_.getMaxNumSplines()) {
      throw std::invalid_argument("Number of splines has to be less than or equal to" + std::to_string(container_.getMaxNumSplines()) +
                                  ".");
    }

    for (const auto& options : optionList) {
      container_.addSpline(PolynomialSplineQuintic(options));
    }
    minTime_ = 0.0;
  }

  void clear() override {
    container_.reset();
    minTime_ = 0.0;
  }

  void transformCurve(const ValueType /*T*/) override { CHECK(false) << "Not implemented"; }

 private:
  SplineContainerType container_;
  Time minTime_;
};

using PolynomialSplineQuinticScalarCurve = PolynomialSplineScalarCurve<PolynomialSplineContainerQuintic>;

} /* namespace curves */
