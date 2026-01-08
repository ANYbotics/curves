#include "curves/polynomial_splines_traits.hpp"

#include <Eigen/Core>
#include <Eigen/QR>

namespace curves {
namespace spline_traits {
//! Map initial pos and final pos to spline coefficients.
bool spline_rep<double, 1>::compute(const SplineOptions& opts, spline_rep<double, 1>::SplineCoefficients& coefficients) {
  coefficients[1] = opts.pos0_;                            // a0
  coefficients[0] = (opts.posT_ - opts.pos0_) / opts.tf_;  // a1

  return true;
}

//! Map initial pos, initial vel and final pos to spline coefficients.
bool spline_rep<double, 2>::compute(const SplineOptions& opts, spline_rep<double, 2>::SplineCoefficients& coefficients) {
  coefficients[2] = opts.pos0_;                                                                 // a0
  coefficients[1] = opts.vel0_;                                                                 // a1
  coefficients[0] = (opts.posT_ - opts.pos0_ - opts.vel0_ * opts.tf_) / std::pow(opts.tf_, 2);  // a2

  return true;
}

//! Map initial pos, initial vel, final pos and final vel to spline coefficients.
bool spline_rep<double, 3>::compute(const SplineOptions& opts, spline_rep<double, 3>::SplineCoefficients& coefficients) {
  static constexpr unsigned int splineOrder = 3;
  static constexpr unsigned int numCoefficients = splineOrder + 1;
  Eigen::Matrix<double, numCoefficients, 1> b;
  b << opts.pos0_, opts.vel0_, opts.posT_, opts.velT_;

  Eigen::Matrix<double, numCoefficients, numCoefficients> A;
  A << Eigen::Map<const Eigen::Matrix<double, 1, numCoefficients>>(tauZero.data()),
      Eigen::Map<const Eigen::Matrix<double, 1, numCoefficients>>(dtauZero.data()),
      Eigen::Map<Eigen::Matrix<double, 1, numCoefficients>>((tau(opts.tf_)).data()),
      Eigen::Map<Eigen::Matrix<double, 1, numCoefficients>>((dtau(opts.tf_)).data());

  Eigen::Map<Eigen::VectorXd>(coefficients.data(), numCoefficients, 1) = A.colPivHouseholderQr().solve(b);

  return true;
}

//! Map initial pos/vel/accel and final pos/vel to spline coefficients.
bool spline_rep<double, 4>::compute(const SplineOptions& opts, spline_rep<double, 4>::SplineCoefficients& coefficients) {
  static constexpr unsigned int splineOrder = 4;
  static constexpr unsigned int numCoefficients = splineOrder + 1;
  Eigen::Matrix<double, numCoefficients, 1> b;
  b << opts.pos0_, opts.vel0_, opts.acc0_, opts.posT_, opts.velT_;

  Eigen::Matrix<double, numCoefficients, numCoefficients> A;
  A << Eigen::Map<const Eigen::Matrix<double, 1, numCoefficients>>(tauZero.data()),
      Eigen::Map<const Eigen::Matrix<double, 1, numCoefficients>>(dtauZero.data()),
      Eigen::Map<const Eigen::Matrix<double, 1, numCoefficients>>(ddtauZero.data()),
      Eigen::Map<Eigen::Matrix<double, 1, numCoefficients>>((tau(opts.tf_)).data()),
      Eigen::Map<Eigen::Matrix<double, 1, numCoefficients>>((dtau(opts.tf_)).data());

  Eigen::Map<Eigen::VectorXd>(coefficients.data(), numCoefficients, 1) = A.colPivHouseholderQr().solve(b);

  return true;
}

//! Map initial pos/vel/accel and final pos/vel/accel to spline coefficients.
bool spline_rep<double, 5>::compute(const SplineOptions& opts, spline_rep<double, 5>::SplineCoefficients& coefficients) {
  static constexpr unsigned int splineOrder = 5;
  static constexpr unsigned int numCoefficients = splineOrder + 1;
  Eigen::Map<Eigen::VectorXd>(coefficients.data(), numCoefficients, 1) =
      (Eigen::Matrix<double, numCoefficients, numCoefficients>()
           << Eigen::Map<const Eigen::Matrix<double, 1, numCoefficients>>(tauZero.data()),
       Eigen::Map<const Eigen::Matrix<double, 1, numCoefficients>>(dtauZero.data()),
       Eigen::Map<const Eigen::Matrix<double, 1, numCoefficients>>(ddtauZero.data()),
       Eigen::Map<Eigen::Matrix<double, 1, numCoefficients>>((tau(opts.tf_)).data()),
       Eigen::Map<Eigen::Matrix<double, 1, numCoefficients>>((dtau(opts.tf_)).data()),
       Eigen::Map<Eigen::Matrix<double, 1, numCoefficients>>((ddtau(opts.tf_)).data()))
          .finished()
          .colPivHouseholderQr()
          .solve((Eigen::Matrix<double, numCoefficients, 1>() << opts.pos0_, opts.vel0_, opts.acc0_, opts.posT_, opts.velT_, opts.accT_)
                     .finished());

  return true;
}

template <typename Core_, int SplineOrder_>
constexpr unsigned int spline_rep<Core_, SplineOrder_>::numCoefficients;

using spline3 = spline_rep<double, 3>;
constexpr std::array<double, spline3::numCoefficients> spline3::tauZero;
constexpr std::array<double, spline3::numCoefficients> spline3::dtauZero;
constexpr std::array<double, spline3::numCoefficients> spline3::ddtauZero;

using spline5 = spline_rep<double, 5>;
constexpr std::array<double, spline5::numCoefficients> spline5::tauZero;
constexpr std::array<double, spline5::numCoefficients> spline5::dtauZero;
constexpr std::array<double, spline5::numCoefficients> spline5::ddtauZero;

}  // namespace spline_traits
}  // namespace curves
