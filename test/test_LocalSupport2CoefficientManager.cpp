/*
 * @file test_LocalSupport2CoefficientManager.cpp
 * @date Aug 17, 2014
 * @author Paul Furgale
 */

#include <cstddef>

#include <gtest/gtest.h>
#include <curves/LocalSupport2CoefficientManager.hpp>

using curves::LocalSupport2CoefficientManager;

class LocalSupport2CoefficientManagerTest : public ::testing::Test {
 protected:
  using Coefficient = Eigen::Matrix<double, 3, 1>;
  using CoefficientIter = LocalSupport2CoefficientManager<Coefficient>::CoefficientIter;

  void SetUp() override {
    numCoefficients_ = 50;
    for (std::size_t i = 0; i < numCoefficients_; ++i) {
      coefficients_.emplace_back(Coefficient::Random(3));
      // Make sure there are some negative times_ in there
      times_.push_back(i * 1000 - 3250);
      keys1_.push_back(manager1_.insertCoefficient(times_[i], coefficients_[i]));
    }
    manager2_.insertCoefficients(times_, coefficients_, &keys2_);

    ASSERT_EQ(numCoefficients_, manager1_.size());
    ASSERT_EQ(numCoefficients_, manager2_.size());

    ASSERT_EQ(numCoefficients_, keys1_.size());
    ASSERT_EQ(numCoefficients_, keys2_.size());

    manager1_.getTimes(&times1_);
    manager2_.getTimes(&times2_);
  }

  std::size_t numCoefficients_{0};
  std::vector<Coefficient> coefficients_;
  std::vector<curves::Time> times_;
  std::vector<curves::Time> times1_;
  std::vector<curves::Time> times2_;
  std::vector<curves::Key> keys1_;
  std::vector<curves::Key> keys2_;
  LocalSupport2CoefficientManager<Coefficient> manager1_;
  LocalSupport2CoefficientManager<Coefficient> manager2_;
};

TEST_F(LocalSupport2CoefficientManagerTest, testInsert) { /* NOLINT */
  ASSERT_EQ(numCoefficients_, manager1_.size());
  ASSERT_EQ(numCoefficients_, manager2_.size());

  ASSERT_EQ(numCoefficients_, keys1_.size());
  ASSERT_EQ(numCoefficients_, keys2_.size());

  ASSERT_EQ(numCoefficients_, times1_.size());
  ASSERT_EQ(numCoefficients_, times2_.size());

  for (size_t i = 0; i < numCoefficients_; ++i) {
    ASSERT_EQ(times1_[i], times_[i]);
    ASSERT_EQ(times2_[i], times_[i]);
  }

  ASSERT_EXIT(manager1_.checkInternalConsistency(true), ::testing::ExitedWithCode(0), "^");
  ASSERT_EXIT(manager2_.checkInternalConsistency(true), ::testing::ExitedWithCode(0), "^");
}

TEST_F(LocalSupport2CoefficientManagerTest, testTimes) { /* NOLINT */
  CoefficientIter bracket0;
  CoefficientIter bracket1;
  bool success = false;
  curves::Time etime{};

  etime = times_[0] - 1;
  success = manager1_.getCoefficientsAt(etime, &bracket0, &bracket1);
  ASSERT_FALSE(success) << "Eval at time " << etime;

  etime = times_[0] - 100;
  success = manager1_.getCoefficientsAt(etime, &bracket0, &bracket1);
  ASSERT_FALSE(success) << "Eval at time " << etime;

  etime = times_[numCoefficients_ - 1];
  success = manager1_.getCoefficientsAt(etime, &bracket0, &bracket1);
  ASSERT_TRUE(success) << "Eval at time " << etime;
  ASSERT_EQ(times_[numCoefficients_ - 2], bracket0->first) << "index " << numCoefficients_ - 2 << ", time: " << etime;
  ASSERT_EQ(times_[numCoefficients_ - 1], bracket1->first) << "index " << numCoefficients_ - 1 << ", time: " << etime;

  etime = times_[numCoefficients_ - 1] + 1;
  success = manager1_.getCoefficientsAt(etime, &bracket0, &bracket1);
  ASSERT_FALSE(success) << "Eval at time " << etime;

  etime = times_[numCoefficients_ - 1] + 100;
  success = manager1_.getCoefficientsAt(etime, &bracket0, &bracket1);
  ASSERT_FALSE(success) << "Eval at time " << etime;

  for (size_t i = 1; i < times_.size(); ++i) {
    etime = times_[i - 1];
    success = manager1_.getCoefficientsAt(etime, &bracket0, &bracket1);
    ASSERT_TRUE(success) << "Eval at time " << etime;
    ASSERT_EQ(times_[i - 1], bracket0->first) << "index " << i << ", time: " << etime;
    ASSERT_EQ(times_[i], bracket1->first) << "index " << i << ", time: " << etime;

    etime = (times_[i - 1] + times_[i]) / 2;
    success = manager1_.getCoefficientsAt(etime, &bracket0, &bracket1);
    ASSERT_TRUE(success) << "Eval at time " << etime;
    ASSERT_EQ(times_[i - 1], bracket0->first) << "index " << i << ", time: " << etime;
    ASSERT_EQ(times_[i], bracket1->first) << "index " << i << ", time: " << etime;
  }
}

TEST_F(LocalSupport2CoefficientManagerTest, testUpdateCoefficients) { /* NOLINT */
  for (auto key : keys1_) {
    manager1_.updateCoefficientByKey(key, Coefficient::Zero());
    ASSERT_EQ(manager1_.getCoefficientByKey(key), Coefficient::Zero());
  }

  typedef boost::unordered_map<curves::Key, Coefficient> CoefficientMap;
  CoefficientMap allCoeffs;
  for (auto key : keys2_) {
    std::pair<curves::Key, Coefficient> pair = std::make_pair(key, Coefficient::Zero());
    allCoeffs.insert(pair);
  }

  manager2_.updateCoefficients(allCoeffs);
  for (auto key : keys2_) {
    ASSERT_EQ(manager2_.getCoefficientByKey(key), Coefficient::Zero());
  }
}
