/*
 * CubicHermiteSE3CurveTest.cpp
 *
 *  Created on: May 27, 2015
 *      Author: Péter Fankhauser, Christian Gehring
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <gtest/gtest.h>

#include <kindr/Core>
#include <kindr/common/gtest_eigen.hpp>

#include "curves/CubicHermiteSE3Curve.hpp"

using namespace curves;

typedef typename curves::CubicHermiteSE3Curve::ValueType VType;

TEST(Evaluate, IdentityPoses) {
  CubicHermiteSE3Curve curve;
  std::vector<curves::Time> times;
  std::vector<VType> values;

  times.push_back(0.0);
  values.push_back(VType());
  times.push_back(1.0);
  values.push_back(VType());
  curve.fitCurve(times, values);

  VType value;

  value.getPosition() = VType::Position(0.2, 3.4, 4.6);
  value.getRotation() = VType::Rotation(kindr::EulerAnglesZyxD(0.3, 2.3, 4.3));
  ASSERT_TRUE(curve.evaluate(value, 0.0));
  EXPECT_EQ(VType::Position(), value.getPosition());
  EXPECT_EQ(VType::Rotation(), value.getRotation());

  value.getPosition() = VType::Position(0.2, 3.4, 4.6);
  value.getRotation() = VType::Rotation(kindr::EulerAnglesZyxD(0.3, 2.3, 4.3));
  ASSERT_TRUE(curve.evaluate(value, 0.5));
  EXPECT_EQ(VType::Position(), value.getPosition());
  EXPECT_EQ(VType::Rotation(), value.getRotation());

  value.getPosition() = VType::Position(0.2, 3.4, 4.6);
  value.getRotation() = VType::Rotation(kindr::EulerAnglesZyxD(0.3, 2.3, 4.3));
  ASSERT_TRUE(curve.evaluate(value, 1.0));
  EXPECT_EQ(VType::Position(), value.getPosition());
  EXPECT_EQ(VType::Rotation(), value.getRotation());
}

TEST(Evaluate, TranslationOnly) {
  CubicHermiteSE3Curve curve;
  std::vector<curves::Time> times;
  std::vector<VType> values;

  times.push_back(0.0);
  values.push_back(VType(VType::Position(-10.0, 10.0, 10.0), VType::Rotation()));
  times.push_back(1.0);
  values.push_back(VType(VType::Position(10.0, -10.0, -10.0), VType::Rotation()));
  curve.fitCurve(times, values);

  VType value;

  ASSERT_TRUE(curve.evaluate(value, 0.0));
  EXPECT_EQ(values[0].getPosition(), value.getPosition());
  EXPECT_EQ(VType::Rotation(), value.getRotation());

  ASSERT_TRUE(curve.evaluate(value, 0.5));
  EXPECT_EQ(VType::Position(0.0, 0.0, 0.0), value.getPosition());
  EXPECT_EQ(VType::Rotation(), value.getRotation());

  ASSERT_TRUE(curve.evaluate(value, 1.0));
  EXPECT_EQ(values[1].getPosition(), value.getPosition());
  EXPECT_EQ(VType::Rotation(), value.getRotation());
}

TEST(InvarianceUnderCoordinateTransformation, Translation) {
  CubicHermiteSE3Curve curve1;
  std::vector<curves::Time> times;
  std::vector<VType> values1;
  times.push_back(0.0);
  values1.push_back(VType(VType::Position(-1.0, 0.1, 10.0), VType::Rotation()));
  times.push_back(5.0);
  values1.push_back(VType(VType::Position(1.0, 0.2, 5.0), VType::Rotation()));
  curve1.fitCurve(times, values1);

  CubicHermiteSE3Curve curve2;
  VType::Position offset(10.0, 10.0, 10.0);
  std::vector<VType> values2;
  for (const auto& value : values1) {
    values2.push_back(value);
    values2.back().getPosition() += offset;
  }
  curve2.fitCurve(times, values2);

  for (double time = times[0]; time <= times[1]; time += 0.1) {
    VType value1, value2;
    ASSERT_TRUE(curve1.evaluate(value1, time));
    ASSERT_TRUE(curve2.evaluate(value2, time));
    std::string msg = std::string{"knot at time: "} + std::to_string(time);
    KINDR_ASSERT_DOUBLE_MX_EQ(value1.getPosition().toImplementation(), (value2.getPosition() - offset).toImplementation(), 1e-3, msg);
    EXPECT_LT(value1.getRotation().getDisparityAngle(value2.getRotation()), 1e-3)
        << "rot1: " << value1.getRotation() << "  rot2: " << value2.getRotation() << std::endl;
  }
}

TEST(InvarianceUnderCoordinateTransformation, Rotation) {
  CubicHermiteSE3Curve curve1;
  std::vector<curves::Time> times;
  std::vector<VType> values1;
  times.push_back(0.0);
  values1.push_back(VType(VType::Position(), VType::Rotation(kindr::EulerAnglesYprPD(0.0, 0.0, 0.0))));
  times.push_back(2.0);
  values1.push_back(VType(VType::Position(), VType::Rotation(kindr::EulerAnglesYprPD(20.0 / 180.0 * M_PI, 0.0, 0.0))));
  times.push_back(4.0);
  values1.push_back(VType(VType::Position(), VType::Rotation(kindr::EulerAnglesYprPD(-20.0 / 180.0 * M_PI, 0.0, 0.0))));
  times.push_back(5.2);
  values1.push_back(VType(VType::Position(), VType::Rotation(kindr::EulerAnglesYprPD(0.0, 0.0, 0.0))));
  curve1.fitCurve(times, values1);

  CubicHermiteSE3Curve curve2;
  VType::Rotation transform(kindr::EulerAnglesYprPD(3.0, 0.0, 0.0));
  std::vector<VType> values2;
  for (const auto& value : values1) {
    values2.push_back(value);
    values2.back().getRotation() = transform * values2.back().getRotation();
    //    values2.back().getRotation().setUnique(); // This results in a flip in w.
  }
  curve2.fitCurve(times, values2);

  for (double time = times[0]; time <= times[3]; time += 0.1) {
    VType value1, value2;
    ASSERT_TRUE(curve1.evaluate(value1, time));
    ASSERT_TRUE(curve2.evaluate(value2, time));
    const VType::Rotation rotation2 = transform.inverted() * value2.getRotation();

    std::string msg = std::string{"knot at time: "} + std::to_string(time);
    KINDR_ASSERT_DOUBLE_MX_EQ(value1.getPosition().toImplementation(), value2.getPosition().toImplementation(), 1e-3, msg);
    EXPECT_LT(value1.getRotation().getDisparityAngle(rotation2), 1e-3)
        << "rot1: " << value1.getRotation() << "  rot2: " << value2.getRotation() << std::endl;
  }
}

TEST(InvarianceUnderCoordinateTransformation, Rotation2) {
  CubicHermiteSE3Curve curve1;
  std::vector<curves::Time> times;
  std::vector<VType> values1;
  times.push_back(0.0);
  values1.push_back(VType(VType::Position(), VType::Rotation(0.0652549, 0.0, 0.0, 0.997869)));
  times.push_back(2.0);
  values1.push_back(VType(VType::Position(), VType::Rotation(0.109015, 0.0, 0.0, -0.99404)));  // w positive.
  times.push_back(4.0);
  values1.push_back(VType(VType::Position(), VType::Rotation(0.237542, 0.0, 0.0, 0.971377)));
  times.push_back(5.2);
  values1.push_back(VType(VType::Position(), VType::Rotation(0.0652549, 0.0, 0.0, 0.997869)));
  curve1.fitCurve(times, values1);

  CubicHermiteSE3Curve curve2;
  VType::Rotation transform = values1.front().getRotation();
  std::vector<VType> values2;
  for (const auto& value : values1) {
    values2.push_back(value);
    values2.back().getRotation() = transform * values2.back().getRotation();
    //    values2.back().getRotation().setUnique(); // Enforces w positive.
  }
  curve2.fitCurve(times, values2);

  for (double time = times[0]; time <= times[3]; time += 0.1) {
    VType value1, value2;
    ASSERT_TRUE(curve1.evaluate(value1, time));
    ASSERT_TRUE(curve2.evaluate(value2, time));
    const VType::Rotation rotation2 = transform.inverted() * value2.getRotation();

    std::string msg = std::string{"knot at time: "} + std::to_string(time);
    KINDR_ASSERT_DOUBLE_MX_EQ(value1.getPosition().toImplementation(), value2.getPosition().toImplementation(), 1e-3, msg);
    EXPECT_LT(value1.getRotation().getDisparityAngle(rotation2), 1e-3)
        << "rot1: " << value1.getRotation() << "  rot2: " << value2.getRotation() << std::endl;
  }
}

TEST(CubicHermiteSE3CurveTest, firstDerivative) {
  CubicHermiteSE3Curve curve;
  std::vector<curves::Time> times;
  std::vector<VType> values;

  double time0 = -1.56;
  times.push_back(time0);
  VType::Rotation rotation0(kindr::EulerAnglesZyxD(M_PI_2, 0.2, -0.9));
  VType::Position position0(1.0, 2.0, 4.0);
  VType transform0(position0, rotation0);
  values.push_back(transform0);

  double timeMid1 = 1.0;
  times.push_back(timeMid1);
  VType::Rotation rotationMid1(kindr::EulerAnglesZyxD(2.0, 3.0, -1.1));
  VType::Position positionMid1(2.0, 4.0, 8.0);
  values.push_back(VType(positionMid1, rotationMid1));

  double timeMid2 = 2.5;
  times.push_back(timeMid2);
  VType::Rotation rotationMid2(kindr::EulerAnglesZyxD(0.2, 0.5, 0.2));
  VType::Position positionMid2(2.0, 4.0, 8.0);
  values.push_back(VType(positionMid2, rotationMid2));

  double timeMid3 = 3.0;
  times.push_back(timeMid3);
  VType::Rotation rotationMid3(kindr::EulerAnglesZyxD(1.5, 0.4, -0.3));
  VType::Position positionMid3(2.0, 4.0, 8.0);
  values.push_back(VType(positionMid3, rotationMid3));

  double time1 = 4.0;
  times.push_back(time1);
  VType::Rotation rotation1(kindr::EulerAnglesZyxD(0.0, 0.0, 0.0));
  VType::Position position1(4.0, 8.0, 16.0);
  values.push_back(VType(position1, rotation1.getUnique()));
  curve.fitCurve(times, values);

  // Check first knot
  VType transform;
  ASSERT_TRUE(curve.evaluate(transform, time0));
  VType expTransform = transform0;
  EXPECT_NEAR(expTransform.getPosition().x(), transform.getPosition().x(), 1e-6);
  EXPECT_NEAR(expTransform.getPosition().y(), transform.getPosition().y(), 1e-6);
  EXPECT_NEAR(expTransform.getPosition().z(), transform.getPosition().z(), 1e-6);
  EXPECT_NEAR(0.0, expTransform.getRotation().getDisparityAngle(transform.getRotation()), 1e-3);

  // Derivative at first knot
  CubicHermiteSE3Curve::DerivativeType derivative;
  ASSERT_TRUE(curve.evaluateDerivative(derivative, time0, 1));
  CubicHermiteSE3Curve::DerivativeType expDerivative;
  KINDR_ASSERT_DOUBLE_MX_EQ(expDerivative.getVector(), derivative.getVector(), 1e-1, "first");

  // Derivative at last knot
  ASSERT_TRUE(curve.evaluateDerivative(derivative, time1, 1));
  expDerivative.setZero();
  KINDR_ASSERT_DOUBLE_MX_EQ(expDerivative.getVector(), derivative.getVector(), 1e-1, "last");

  // Finite difference
  double h = 1.0e-8;
  for (double time = times[0] + h; time <= times.back(); time += 0.1) {
    double timeA = time - h;
    double timeB = time + h;
    VType T_A, T_B;
    ASSERT_TRUE(curve.evaluate(T_A, timeA));
    ASSERT_TRUE(curve.evaluate(T_B, timeB));
    Eigen::Vector3d angularVel = T_B.getRotation().boxMinus(T_A.getRotation()) / (2.0 * h);
    Eigen::Vector3d linearVel = (T_B.getPosition().vector() - T_A.getPosition().vector()) / (2.0 * h);
    expDerivative = CubicHermiteSE3Curve::DerivativeType(linearVel, angularVel);
    ASSERT_TRUE(curve.evaluateDerivative(derivative, time, 1));
    KINDR_ASSERT_DOUBLE_MX_EQ_ZT(expDerivative.getVector(), derivative.getVector(), 1.0, "fd", 1.0e-7);
  }
}

TEST(CubicHermiteSE3CurveTest, linearAcceleration) {
  CubicHermiteSE3Curve curve;
  std::vector<curves::Time> times;
  std::vector<VType> values;

  double time0 = -1.56;
  times.push_back(time0);
  VType::Rotation rotation0(kindr::EulerAnglesZyxD(M_PI_2, 0.2, -0.9));
  VType::Position position0(1.0, 2.0, 4.0);
  VType transform0(position0, rotation0);
  values.push_back(transform0);

  double timeMid1 = 1.0;
  times.push_back(timeMid1);
  VType::Rotation rotationMid1(kindr::EulerAnglesZyxD(2.0, 3.0, -1.1));
  VType::Position positionMid1(2.0, 4.0, 8.0);
  values.push_back(VType(positionMid1, rotationMid1));

  double timeMid2 = 2.5;
  times.push_back(timeMid2);
  VType::Rotation rotationMid2(kindr::EulerAnglesZyxD(0.2, 0.5, 0.2));
  VType::Position positionMid2(2.0, 4.0, 8.0);
  values.push_back(VType(positionMid2, rotationMid2));

  double timeMid3 = 3.0;
  times.push_back(timeMid3);
  VType::Rotation rotationMid3(kindr::EulerAnglesZyxD(1.5, 0.4, -0.3));
  VType::Position positionMid3(2.0, 4.0, 8.0);
  values.push_back(VType(positionMid3, rotationMid3));

  double time1 = 4.0;
  times.push_back(time1);
  VType::Rotation rotation1(kindr::EulerAnglesZyxD(0.0, 0.0, 0.0));
  VType::Position position1(4.0, 8.0, 16.0);
  values.push_back(VType(position1, rotation1.getUnique()));

  curve.fitCurve(times, values);

  // Finite difference
  double h = 1.0e-8;
  for (double time = times[0] + h; time <= times.back(); time += 0.1) {
    double timeA = time - h;
    double timeB = time + h;
    CubicHermiteSE3Curve::DerivativeType d_W_A, d_W_B;
    ASSERT_TRUE(curve.evaluateDerivative(d_W_A, timeA, 1));
    ASSERT_TRUE(curve.evaluateDerivative(d_W_B, timeB, 1));
    Eigen::Vector3d expLinAcc = (d_W_B.getTranslationalVelocity().vector() - d_W_A.getTranslationalVelocity().vector()) / (2.0 * h);

    kindr::Acceleration3D linAcc;
    ASSERT_TRUE(curve.evaluateLinearAcceleration(linAcc, time));
    KINDR_ASSERT_DOUBLE_MX_EQ_ZT(expLinAcc, linAcc.vector(), 1.0, "fd", 1.0e-7);
  }
}

TEST(Debugging, FreeGaitTorsoControl) {
  CubicHermiteSE3Curve curve;
  std::vector<curves::Time> times;
  std::vector<VType> values;
  VType::Rotation fixRotation(0.999999, -6.31036e-05, 0.00109732, -3.43683e-06);
  //  fixRotation.setIdentity(); // With this it succeeds.
  times.push_back(0.0);
  values.push_back(VType(VType::Position(99.9, 0.0, 0.38), fixRotation));
  times.push_back(1.5);
  values.push_back(VType(VType::Position(100.0, 0.0, 0.40), fixRotation));
  curve.fitCurve(times, values);

  //  curve.print();

  for (double time = times[0]; time <= times[1]; time += 0.1) {
    VType value;
    ASSERT_TRUE(curve.evaluate(value, time));
    const VType::Position position = value.getPosition();
    const VType::Rotation rotation = value.getRotation();

    //    std::cout << "curves::Time: " << time << " s, Position: " << position << std::endl;
    //    std::cout << "curves::Time: " << time << " s, Rotation: " << rotation << std::endl;

    EXPECT_GE(position.x(), values[0].getPosition().x());
    //    EXPECT_GE(position.y(), values[0].getPosition().y()); // Numerical issues.
    EXPECT_GE(position.z(), values[0].getPosition().z());
    EXPECT_LE(position.x(), values[1].getPosition().x());
    //    EXPECT_LE(position.y(), values[1].getPosition().y()); // Numerical issues.
    EXPECT_LE(position.z(), values[1].getPosition().z());

    EXPECT_NEAR(fixRotation.x(), rotation.x(), 1e-6);
    EXPECT_NEAR(fixRotation.y(), rotation.y(), 1e-6);
    EXPECT_NEAR(fixRotation.z(), rotation.z(), 1e-6);
    EXPECT_NEAR(fixRotation.w(), rotation.w(), 1e-6);
  }
}

TEST(GetTime, Simple) {
  CubicHermiteSE3Curve curve;
  std::vector<curves::Time> times;
  std::vector<VType> values;

  times.push_back(1.1);
  values.push_back(VType());
  times.push_back(5.6);
  values.push_back(VType());
  times.push_back(7.2);
  values.push_back(VType());
  curve.fitCurve(times, values);

  EXPECT_EQ(times[0], curve.getMinTime());
  EXPECT_EQ(times[2], curve.getMaxTime());
}
