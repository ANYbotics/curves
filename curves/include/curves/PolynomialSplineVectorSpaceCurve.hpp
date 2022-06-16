/*
 * PolynomialSplineVectorSpaceCurve.hpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <glog/logging.h>
#include <Eigen/Core>
#include <string>
#include <vector>

#include "curves/Curve.hpp"
#include "curves/PolynomialSplineContainer.hpp"
#include "curves/VectorSpaceCurve.hpp"
#include "curves/polynomial_splines_containers.hpp"

namespace curves {

template <typename SplineType, int N>
class PolynomialSplineVectorSpaceCurve : public VectorSpaceCurve<N> {
 public:
  using Parent = VectorSpaceCurve<N>;
  using ValueType = typename Parent::ValueType;
  using DerivativeType = typename Parent::DerivativeType;

  PolynomialSplineVectorSpaceCurve() : VectorSpaceCurve<N>(), minTime_(0) { containers_.resize(N); }

  void print(const std::string& /*str*/) const override {}

  Time getMinTime() const override { return minTime_; }

  Time getMaxTime() const override { return containers_.at(0).getContainerDuration(); }

  bool evaluate(ValueType& value, Time time) const override {
    for (size_t i = 0; i < N; ++i) {
      value(i) = containers_.at(i).getPositionAtTime(time);
    }
    return true;
  }

  bool evaluateDerivative(DerivativeType& value, Time time, unsigned derivativeOrder) const override {
    for (size_t i = 0; i < N; ++i) {
      if (derivativeOrder == 1) {
        value(i) = containers_.at(i).getVelocityAtTime(time);
      } else if (derivativeOrder == 2) {
        value(i) = containers_.at(i).getAccelerationAtTime(time);
      } else {
        return false;
      }
    }
    return true;
  }

  void extend(const std::vector<Time>& /*times*/, const std::vector<ValueType>& /*values*/, std::vector<Key>* /*outKeys*/) override {
    throw std::runtime_error("PolynomialSplineVectorSpaceCurve::extend is not yet implemented!");
  }

  void fitCurve(const std::vector<Time>& times, const std::vector<ValueType>& values, std::vector<Key>* /*outKeys*/) override {
    minTime_ = times.front();
    for (size_t i = 0; i < N; ++i) {
      std::vector<double> scalarValues;
      scalarValues.reserve(times.size());
      for (size_t t = 0; t < times.size(); ++t) {
        scalarValues.push_back(values.at(t)(i));
      }
      containers_.at(i).setData(times, scalarValues, 0.0, 0.0, 0.0, 0.0);
    }
  }

  virtual void fitCurve(const std::vector<Time>& times, const std::vector<ValueType>& values, const DerivativeType& initialVelocity,
                        const DerivativeType& initialAcceleration, const DerivativeType& finalVelocity,
                        const DerivativeType& finalAcceleration) {
    minTime_ = times.front();
    for (size_t i = 0; i < N; ++i) {
      std::vector<double> scalarValues;
      scalarValues.reserve(times.size());
      for (size_t t = 0; t < times.size(); ++t) {
        scalarValues.push_back(values.at(t)(i));
      }
      containers_.at(i).setData(times, scalarValues, initialVelocity(i), initialAcceleration(i), finalVelocity(i), finalAcceleration(i));
    }
  }

  virtual void fitCurve(const std::vector<Time>& times, const std::vector<ValueType>& values,
                        const std::vector<DerivativeType>& firstDerivatives, const std::vector<DerivativeType>& secondDerivatives,
                        std::vector<Key>* /*outKeys*/) {
    minTime_ = times.front();
    for (size_t i = 0; i < N; ++i) {
      std::vector<double> scalarValues;
      std::vector<double> scalarFirstDerivates;
      std::vector<double> scalarSecondDerivates;
      scalarValues.reserve(times.size());
      scalarFirstDerivates.reserve(times.size());
      scalarSecondDerivates.reserve(times.size());
      for (size_t t = 0; t < times.size(); ++t) {
        scalarValues.push_back(values.at(t)(i));
        scalarFirstDerivates.push_back(firstDerivatives.at(t)(i));
        scalarSecondDerivates.push_back(secondDerivatives.at(t)(i));
      }
      // TODO(anyone): Copy all derivatives, right now only first and last are supported.
      containers_.at(i).setData(times, scalarValues, *(scalarFirstDerivates.begin()), *(scalarSecondDerivates.begin()),
                                *(scalarFirstDerivates.end() - 1), *(scalarSecondDerivates.end() - 1));
    }
  }

  virtual void fitCurve(const std::vector<SplineOptions>& /*values*/, std::vector<Key>* /*outKeys*/) {
    throw std::runtime_error("PolynomialSplineVectorSpaceCurve::fitCurve is not yet implemented!");
  }

  void clear() override {
    for (size_t i = 0; i < N; ++i) {
      containers_.at(i).reset();
    }
  }

  void transformCurve(const ValueType /*T*/) override { CHECK(false) << "Not implemented"; }

 private:
  std::vector<PolynomialSplineContainerQuintic> containers_;
  Time minTime_;
};

using PolynomialSplineQuinticVector3Curve = PolynomialSplineVectorSpaceCurve<PolynomialSplineQuintic, 3>;

}  // namespace curves
