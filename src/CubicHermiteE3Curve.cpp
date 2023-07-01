/*
 * CubicHermiteE3Curve.cpp
 *
 *  Created on: Aug 16, 2016
 *      Author: gehrinch
 */

#include <curves/CubicHermiteE3Curve.hpp>

namespace curves {

/// Print the value of the coefficient, for debugging and unit tests
void CubicHermiteE3Curve::print(const std::string& str) const {
  std::cout << "=========================================" << std::endl;
  std::cout << "======= Cubic Hermite SE3 CURVE =========" << std::endl;
  std::cout << str << std::endl;
  std::cout << "num of coefficients: " << manager_.size() << std::endl;
  std::cout << "dimension: " << 6 << std::endl;
  std::vector<Key> keys;
  std::vector<Time> times;
  manager_.getTimes(&times);
  manager_.getKeys(&keys);
  std::cout << "curve defined between times: " << manager_.getMinTime() << " and " << manager_.getMaxTime() << std::endl;
  std::cout << "=========================================" << std::endl;
  for (size_t i = 0; i < manager_.size(); i++) {
    std::cout << "coefficient " << keys[i] << ": ";
    std::cout << manager_.getCoefficientByKey(keys[i]).getPosition() << std::endl;
    //    gtsam::traits<Coefficient>::Print(manager_.getCoefficientByKey(keys[i]),ss.str());
    std::cout << " | time: " << times[i];
    std::cout << std::endl;
  }
  std::cout << "=========================================" << std::endl;
}

bool CubicHermiteE3Curve::writeEvalToFile(const std::string& filename, int nSamples) const {
  FILE* fp = fopen(filename.c_str(), "w");
  if (fp == nullptr) {
    std::cout << "Could not open file to write" << std::endl;
    return false;
  }
  fprintf(fp, "t ");
  fprintf(fp, "px py pz ");
  fprintf(fp, "rw rx ry rz ");
  fprintf(fp, "vx vy vz ");
  fprintf(fp, "wx wy wz ");
  fprintf(fp, "\n");

  Time dt = (getMaxTime() - getMinTime()) / (nSamples - 1);
  ValueType position;
  DerivativeType velocity;
  Time t = getMinTime();
  while (true) {
    if (t > getMaxTime()) {
      break;
    }
    if (!evaluate(position, t)) {
      std::cout << "Could not evaluate at time " << t << std::endl;
      fclose(fp);
      return false;
    }
    if (!evaluateDerivative(velocity, t, 1)) {
      std::cout << "Could not evaluate derivative at time " << t << std::endl;
      fclose(fp);
      return false;
    }
    fprintf(fp, "%lf ", t);
    fprintf(fp, "%lf %lf %lf ", position.x(), position.y(), position.z());
    fprintf(fp, "%lf %lf %lf ", velocity.x(), velocity.y(), velocity.z());
    fprintf(fp, "\n");
    t += dt;
  }
  fclose(fp);

  return true;
}

/// The first valid time for the curve.
Time CubicHermiteE3Curve::getMinTime() const {
  return manager_.getMinTime();
}

/// The one past the last valid time for the curve.
Time CubicHermiteE3Curve::getMaxTime() const {
  return manager_.getMaxTime();
}

bool CubicHermiteE3Curve::isEmpty() const {
  std::vector<Time> outTimes;
  manager_.getTimes(&outTimes);
  return outTimes.empty();
}

// return number of coefficients curve is composed of
int CubicHermiteE3Curve::size() const {
  return manager_.size();
}

void CubicHermiteE3Curve::extend(const std::vector<Time>& /*times*/, const std::vector<ValueType>& /*values*/,
                                 std::vector<Key>* /*outKeys*/) {}

void CubicHermiteE3Curve::fitCurve(const std::vector<Time>& times, const std::vector<ValueType>& values, std::vector<Key>* outKeys) {
  fitCurveWithDerivatives(times, values, DerivativeType::Zero(), DerivativeType::Zero(), outKeys);
}

void CubicHermiteE3Curve::fitPeriodicCurve(const std::vector<Time>& times, const std::vector<ValueType>& values,
                                           std::vector<Key>* outKeys) {
  // Note: We assume that the first and last points coincide.
  const size_t nPoints = times.size();
  // TODO(anyone) Add a check on nPoints which should be >= 2 otherwise that breaks.
  DerivativeType derivative = calculateSlope(times[nPoints - 2], times[1], values[nPoints - 2], values[1]);
  fitCurveWithDerivatives(times, values, derivative, derivative, outKeys);
}

void CubicHermiteE3Curve::fitCurveWithDerivatives(const std::vector<Time>& times, const std::vector<ValueType>& values,
                                                  const DerivativeType& initialDerivative, const DerivativeType& finalDerivative,
                                                  std::vector<Key>* outKeys) {
  assert(times.size() == values.size());

  // construct the Hemrite coefficients
  std::vector<Coefficient> coefficients;
  // fill the coefficients with ValueType and DerivativeType
  // use Catmull-Rom interpolation for derivatives on knot points
  for (size_t i = 0; i < times.size(); ++i) {
    DerivativeType derivative;
    // Treat the boundaries (i == 0 && i == max) different from the internal points
    if (i == 0) {
      // First key.
      derivative = initialDerivative;
    } else if (i == times.size() - 1) {
      // Last key.
      derivative = finalDerivative;
    } else {
      // Other keys.
      derivative = calculateSlope(times[i - 1], times[i + 1], values[i - 1], values[i + 1]);
    }

    coefficients.emplace_back(values[i], derivative);
  }

  manager_.insertCoefficients(times, coefficients, outKeys);
}

/// Evaluate the ambient space of the curve.
bool CubicHermiteE3Curve::evaluate(ValueType& value, Time time) const {
  // Check if the curve is only defined at this one time
  if (manager_.getMaxTime() == time && manager_.getMinTime() == time) {
    value = manager_.coefficientBegin()->second.coefficient_.getPosition();
    return true;
  } else {
    CoefficientIter a;
    CoefficientIter b;
    bool success = manager_.getCoefficientsAt(time, &a, &b);
    if (!success) {
      std::cerr << "Unable to get the coefficients at time " << time << std::endl;
      return false;
    }

    // read out transformation from coefficient
    const ValueType T_W_A = a->second.coefficient_.getPosition();
    const ValueType T_W_B = b->second.coefficient_.getPosition();

    // read out derivative from coefficient
    const DerivativeType d_W_A = a->second.coefficient_.getVelocity();
    const DerivativeType d_W_B = b->second.coefficient_.getVelocity();

    // make alpha
    const double dt_sec = (b->first - a->first);
    const double alpha = double(time - a->first) / (b->first - a->first);

    // Implemantation of Hermite Interpolation not easy and not fun (without expressions)!

    // translational part (easy):
    const double alpha2 = alpha * alpha;
    const double alpha3 = alpha2 * alpha;

    const double beta0 = 2.0 * alpha3 - 3.0 * alpha2 + 1.0;
    const double beta1 = -2.0 * alpha3 + 3.0 * alpha2;
    const double beta2 = alpha3 - 2.0 * alpha2 + alpha;
    const double beta3 = alpha3 - alpha2;

    /**************************************************************************************
     *  Translational part:
     **************************************************************************************/
    value = ValueType(T_W_A * beta0 + T_W_B * beta1 + d_W_A * (beta2 * dt_sec) + d_W_B * (beta3 * dt_sec));
    return true;
  }
  return false;
}

/// Evaluate the curve derivatives.
bool CubicHermiteE3Curve::evaluateDerivative(DerivativeType& derivative, Time time, unsigned int derivativeOrder) const {
  if (derivativeOrder == 1) {
    // Check if the curve is only defined at this one time
    if (manager_.getMaxTime() == time && manager_.getMinTime() == time) {
      derivative = manager_.coefficientBegin()->second.coefficient_.getVelocity();
      return true;
    } else {
      CoefficientIter a;
      CoefficientIter b;
      bool success = manager_.getCoefficientsAt(time, &a, &b);
      if (!success) {
        std::cerr << "Unable to get the coefficients at time " << time << std::endl;
        return false;
      }

      // read out transformation from coefficient
      const ValueType T_W_A = a->second.coefficient_.getPosition();
      const ValueType T_W_B = b->second.coefficient_.getPosition();

      // read out derivative from coefficient
      const DerivativeType d_W_A = a->second.coefficient_.getVelocity();
      const DerivativeType d_W_B = b->second.coefficient_.getVelocity();

      // make alpha
      double dt_sec = (b->first - a->first);
      const double one_over_dt_sec = 1.0 / dt_sec;
      double alpha = double(time - a->first) / dt_sec;

      const double alpha2 = alpha * alpha;
      // const double alpha3 = alpha2 * alpha; // TODO(maintainer) This value is unused - maybe a bug?

      /**************************************************************************************
       *  Translational part:
       **************************************************************************************/
      // Implementation of translation
      const double gamma0 = 6.0 * (alpha2 - alpha);
      const double gamma1 = 3.0 * alpha2 - 4.0 * alpha + 1.0;
      const double gamma2 = 6.0 * (alpha - alpha2);
      const double gamma3 = 3.0 * alpha2 - 2.0 * alpha;

      const DerivativeType velocity_m_s =
          T_W_A * (gamma0 * one_over_dt_sec) + d_W_A * (gamma1) + T_W_B * (gamma2 * one_over_dt_sec) + d_W_B * (gamma3);

      derivative = velocity_m_s;
      return true;
    }
  } else if (derivativeOrder == 2) {
    return evaluateLinearAcceleration(derivative, time);
  } else {
    std::cerr << "CubicHermiteSE3Curve::evaluateDerivative: higher order derivatives are not implemented!";
    return false;
  }
}

bool CubicHermiteE3Curve::evaluateLinearAcceleration(Acceleration& linearAcceleration, Time time) const {
  CoefficientIter a;
  CoefficientIter b;
  bool success = manager_.getCoefficientsAt(time, &a, &b);
  if (!success) {
    std::cerr << "Unable to get the coefficients at time " << time << std::endl;
    return false;
  }

  // read out transformation from coefficient
  const ValueType T_W_A = a->second.coefficient_.getPosition();
  const ValueType T_W_B = b->second.coefficient_.getPosition();

  // read out derivative from coefficient
  const DerivativeType d_W_A = a->second.coefficient_.getVelocity();
  const DerivativeType d_W_B = b->second.coefficient_.getVelocity();

  // make alpha
  double dt_sec = (b->first - a->first);
  const double one_over_dt_sec = 1.0 / dt_sec;
  double alpha = double(time - a->first) / dt_sec;
  const double d_alpha = one_over_dt_sec;

  /**************************************************************************************
   *  Translational part:
   **************************************************************************************/
  // Implementation of translation
  const double d_gamma0 = 6.0 * (2 * alpha - 1.0) * d_alpha;
  const double d_gamma1 = (6.0 * alpha - 4.0) * d_alpha;
  const double d_gamma2 = 6.0 * (1.0 - 2.0 * alpha) * d_alpha;
  const double d_gamma3 = (6.0 * alpha - 2.0) * d_alpha;

  linearAcceleration =
      Acceleration(T_W_A * d_gamma0 * one_over_dt_sec + d_W_A * d_gamma1 + T_W_B * d_gamma2 * one_over_dt_sec + d_W_B * d_gamma3);

  return true;
}

void CubicHermiteE3Curve::clear() {
  manager_.clear();
}

}  // namespace curves