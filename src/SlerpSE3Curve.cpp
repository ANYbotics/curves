/*
 * SlerpSE3Curve.cpp
 *
 *  Created on: Oct 10, 2014
 *      Author: Renaud Dube, Abel Gawel, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <curves/SlerpSE3Curve.hpp>
#include <iostream>

namespace curves {

void SlerpSE3Curve::print(const std::string& str) const {
  std::cout << "=========================================" << std::endl;
  std::cout << "=========== Slerp SE3 CURVE =============" << std::endl;
  std::cout << str << std::endl;
  std::cout << "num of coefficients: " << manager_.size() << std::endl;
  std::cout << "dimension: " << 6 << std::endl;
  std::stringstream ss;
  std::vector<Key> keys;
  std::vector<Time> times;
  manager_.getTimes(&times);
  manager_.getKeys(&keys);
  std::cout << "curve defined between times: " << manager_.getMinTime() << " and " << manager_.getMaxTime() << std::endl;
  double sum_dp = 0;
  Eigen::Vector3d p1;
  Eigen::Vector3d p2;
  for (size_t i = 0; i < times.size() - 1; ++i) {
    p1 = evaluate(times[i]).getPosition().vector();
    p2 = evaluate(times[i + 1]).getPosition().vector();
    sum_dp += (p1 - p2).norm();
  }
  std::cout << "average dt between coefficients: "
            << (manager_.getMaxTime() - manager_.getMinTime()) / static_cast<double>(times.size() - 1) << " ns." << std::endl;
  std::cout << "average distance between coefficients: " << sum_dp / double((times.size() - 1)) << " m." << std::endl;
  std::cout << "=========================================" << std::endl;
  for (size_t i = 0; i < manager_.size(); i++) {
    ss << "coefficient " << keys[i] << ": ";
    std::cout << " | time: " << times[i];
    std::cout << std::endl;
    ss.str("");
  }
  std::cout << "=========================================" << std::endl;
}

Time SlerpSE3Curve::getMaxTime() const {
  return manager_.getMaxTime();
}

Time SlerpSE3Curve::getMinTime() const {
  return manager_.getMinTime();
}

bool SlerpSE3Curve::isEmpty() const {
  return manager_.empty();
}

int SlerpSE3Curve::size() const {
  return manager_.size();
}

void SlerpSE3Curve::fitCurve(const std::vector<Time>& times, const std::vector<ValueType>& values, std::vector<Key>* outKeys) {
  assert(times.size() == values.size());
  if (!times.empty()) {
    clear();
    manager_.insertCoefficients(times, values, outKeys);
  }
}

void SlerpSE3Curve::setCurve(const std::vector<Time>& times, const std::vector<ValueType>& values) {
  assert(times.size() == values.size());
  if (!times.empty()) {
    manager_.insertCoefficients(times, values);
  }
}

void SlerpSE3Curve::extend(const std::vector<Time>& times, const std::vector<ValueType>& values, std::vector<Key>* outKeys) {
  if (times.size() != values.size()) {
    assert(times.size() == values.size() && "Number of times and number of coefficients don't match.");
  }

  slerpPolicy_.extend<SlerpSE3Curve, ValueType>(times, values, this, outKeys);
}

typename SlerpSE3Curve::DerivativeType SlerpSE3Curve::evaluateDerivative(Time /*time*/, unsigned /*derivativeOrder*/) const {
  assert(false && "Not implemented");
  return SlerpSE3Curve::DerivativeType{};
}

/// \brief \f[T^{\alpha}\f]
SE3 transformationPower(SE3 /*T*/, double /*alpha*/) {
  assert(false && "Not implemented");
  return SE3{};
}

/// \brief \f[A*B\f]
SE3 composeTransformations(SE3 A, SE3 B) {
  return A * B;
}

/// \brief \f[T^{-1}\f]
SE3 inverseTransformation(SE3 /*T*/) {
  assert(false && "Not implemented");
  return SE3{};
}

SE3 invertAndComposeImplementation(SE3 A, SE3 B) {
  SE3 result = composeTransformations(inverseTransformation(A), B);
  return result;
}

SE3 SlerpSE3Curve::evaluate(Time /*time*/) const {
  assert(false && "Not implemented");
  return SE3{};
}

void SlerpSE3Curve::setTimeRange(Time /*minTime*/, Time /*maxTime*/) {
  // \todo Abel and Renaud
  assert(false && "Not implemented");
}

/// \brief Evaluate the angular velocity of Frame b as seen from Frame a, expressed in Frame a.
Eigen::Vector3d SlerpSE3Curve::evaluateAngularVelocityA(Time /*time*/) {
  assert(false && "Not implemented");
  return Eigen::Vector3d{};
}
/// \brief Evaluate the angular velocity of Frame a as seen from Frame b, expressed in Frame b.
Eigen::Vector3d SlerpSE3Curve::evaluateAngularVelocityB(Time /*time*/) {
  assert(false && "Not implemented");
  return Eigen::Vector3d{};
}
/// \brief Evaluate the velocity of Frame b as seen from Frame a, expressed in Frame a.
Eigen::Vector3d SlerpSE3Curve::evaluateLinearVelocityA(Time /*time*/) {
  assert(false && "Not implemented");
  return Eigen::Vector3d{};
}
/// \brief Evaluate the velocity of Frame a as seen from Frame b, expressed in Frame b.
Eigen::Vector3d SlerpSE3Curve::evaluateLinearVelocityB(Time /*time*/) {
  assert(false && "Not implemented");
  return Eigen::Vector3d{};
}
/// \brief evaluate the velocity/angular velocity of Frame b as seen from Frame a,
/// expressed in Frame a. The return value has the linear velocity (0,1,2),
/// and the angular velocity (3,4,5).
Vector6d SlerpSE3Curve::evaluateTwistA(Time /*time*/) {
  assert(false && "Not implemented");
  return Vector6d{};
}
/// \brief evaluate the velocity/angular velocity of Frame a as seen from Frame b,
/// expressed in Frame b. The return value has the linear velocity (0,1,2),
/// and the angular velocity (3,4,5).
Vector6d SlerpSE3Curve::evaluateTwistB(Time /*time*/) {
  assert(false && "Not implemented");
  return Vector6d{};
}
/// \brief Evaluate the angular derivative of Frame b as seen from Frame a, expressed in Frame a.
Eigen::Vector3d SlerpSE3Curve::evaluateAngularDerivativeA(unsigned /*derivativeOrder*/, Time /*time*/) {
  assert(false && "Not implemented");
  return Eigen::Vector3d{};
}
/// \brief Evaluate the angular derivative of Frame a as seen from Frame b, expressed in Frame b.
Eigen::Vector3d SlerpSE3Curve::evaluateAngularDerivativeB(unsigned /*derivativeOrder*/, Time /*time*/) {
  assert(false && "Not implemented");
  return Eigen::Vector3d{};
}
/// \brief Evaluate the derivative of Frame b as seen from Frame a, expressed in Frame a.
Eigen::Vector3d SlerpSE3Curve::evaluateLinearDerivativeA(unsigned /*derivativeOrder*/, Time /*time*/) {
  assert(false && "Not implemented");
  return Eigen::Vector3d{};
}
/// \brief Evaluate the derivative of Frame a as seen from Frame b, expressed in Frame b.
Eigen::Vector3d SlerpSE3Curve::evaluateLinearDerivativeB(unsigned /*derivativeOrder*/, Time /*time*/) {
  assert(false && "Not implemented");
  return Eigen::Vector3d{};
}
/// \brief evaluate the velocity/angular derivative of Frame b as seen from Frame a,
/// expressed in Frame a. The return value has the linear velocity (0,1,2),
/// and the angular velocity (3,4,5).
Vector6d SlerpSE3Curve::evaluateDerivativeA(unsigned /*derivativeOrder*/, Time /*time*/) {
  assert(false && "Not implemented");
  return Vector6d{};
}
/// \brief evaluate the velocity/angular velocity of Frame a as seen from Frame b,
/// expressed in Frame b. The return value has the linear velocity (0,1,2),
/// and the angular velocity (3,4,5).
Vector6d SlerpSE3Curve::evaluateDerivativeB(unsigned /*derivativeOrder*/, Time /*time*/) {
  assert(false && "Not implemented");
  return Vector6d{};
}

void SlerpSE3Curve::setMinSamplingPeriod(Time time) {
  slerpPolicy_.setMinSamplingPeriod(time);
}

///   eg. 4 will add a coefficient every 4 extend
void SlerpSE3Curve::setSamplingRatio(const int ratio) {
  slerpPolicy_.setMinimumMeasurements(ratio);
}

void SlerpSE3Curve::clear() {
  manager_.clear();
}

void SlerpSE3Curve::transformCurve(const ValueType T) {
  std::vector<Time> coefTimes;
  manager_.getTimes(&coefTimes);
  for (auto coefTime : coefTimes) {
    // Apply a rigid transformation to every coefficient (on the left side).
    manager_.insertCoefficient(coefTime, T * evaluate(coefTime));
  }
}

void SlerpSE3Curve::saveCurveTimesAndValues(const std::string& filename) const {
  std::vector<Time> curveTimes;
  manager_.getTimes(&curveTimes);

  saveCurveAtTimes(filename, curveTimes);
}

void SlerpSE3Curve::saveCurveAtTimes(const std::string& filename, std::vector<Time> times) const {
  Eigen::VectorXd v(7);

  std::vector<Eigen::VectorXd> curveValues;
  ValueType val;
  for (auto time : times) {
    val = evaluate(time);
    v << val.getPosition().x(), val.getPosition().y(), val.getPosition().z(), val.getRotation().w(), val.getRotation().x(),
        val.getRotation().y(), val.getRotation().z();
    curveValues.push_back(v);
  }

  writeTimeVectorCSV(filename, times, curveValues);
}

void SlerpSE3Curve::getCurveTimes(std::vector<Time>* outTimes) const {
  manager_.getTimes(outTimes);
}

}  // namespace curves
