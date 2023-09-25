#pragma once

#include <string>
#include <vector>

#include "curves/Curve.hpp"
#include "curves/PolynomialSplineContainer.hpp"

namespace curves {

struct ScalarCurveConfig {
  using ValueType = double;
  using DerivativeType = double;
};

template <typename SplineContainerType_>
class PolynomialSplineScalarCurve : public Curve<ScalarCurveConfig> {
 public:
  using Parent = Curve<ScalarCurveConfig>;
  using ValueType = typename Parent::ValueType;
  using DerivativeType = typename Parent::DerivativeType;

  using SplineContainerType = SplineContainerType_;

  PolynomialSplineScalarCurve() : Parent(), container_(), minTime_(0.0) {}

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
    if (optionList.size() > static_cast<std::size_t>(container_.getMaxNumSplines())) {
      throw std::invalid_argument("Number of splines has to be less than or equal to" + std::to_string(container_.getMaxNumSplines()) +
                                  ".");
    }

    for (const auto& options : optionList) {
      container_.addSpline(PolynomialSpline<5>(options));
    }
    minTime_ = 0.0;
  }

  void clear() override {
    container_.reset();
    minTime_ = 0.0;
  }

 private:
  SplineContainerType container_;
  Time minTime_;
};

using PolynomialSplineQuinticScalarCurve = PolynomialSplineScalarCurve<PolynomialSplineContainer<5>>;

} /* namespace curves */
