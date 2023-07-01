/*
 * LocalSupport2CoefficientManager.hpp
 *
 *  Created on: Oct 10, 2014
 *      Author: Paul Furgale, Abel Gawel, Renaud Dube, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <cstddef>
#include <map>
#include <utility>
#include <vector>

#include <boost/unordered_map.hpp>

#include <Eigen/Core>

#include "curves/Curve.hpp"

namespace curves {

using Key = std::size_t;

template <class Coefficient>
class LocalSupport2CoefficientManager {
 public:
  using CoefficientType = Coefficient;

  struct KeyCoefficient {
    Key key_{0};
    CoefficientType coefficient_;

    KeyCoefficient(const Key key, Coefficient coefficient) : key_(key), coefficient_(std::move(coefficient)) {}
    KeyCoefficient() = default;

    bool equals(const KeyCoefficient& other) const {
      // Note: here we assume that == operator is implemented by the coefficient.
      // Could not use gtsam traits as the gtsam namespace is not visible to this class.
      // Is this correct?
      return key_ == other.key && coefficient_ == other.coefficient;
    }

    bool operator==(const KeyCoefficient& other) const { return this->equals(other); }
  };

  using TimeToKeyCoefficientMap = std::map<Time, KeyCoefficient>;
  using CoefficientIter = typename TimeToKeyCoefficientMap::const_iterator;

  /// Key/Coefficient pairs
  using CoefficientMap = boost::unordered_map<size_t, Coefficient>;

  /// Compare this Coefficient manager with another for equality.
  bool equals(const LocalSupport2CoefficientManager& other) const;

  /// Print the value of the coefficient, for debugging and unit tests
  void print(const std::string& str = "") const;

  /// Get all of the keys in this manager. This method clears the
  /// list of keys before filling it.
  void getKeys(std::vector<Key>* outKeys) const;

  /// Get all of the keys in this manager. The list is not cleared
  /// before pushing it to the container.
  void appendKeys(std::vector<Key>* outKeys) const;

  /// Get a sorted list of coefficient times
  void getTimes(std::vector<Time>* outTimes) const;

  /// Get a sorted list of coefficient times in a given time window
  void getTimesInWindow(std::vector<Time>* outTimes, Time begTime, Time endTime) const;

  /// Modify multiple coefficient values. Time is assumed to be ordered.
  void modifyCoefficientsValuesInBatch(const std::vector<Time>& times, const std::vector<Coefficient>& values);

  /// \brief insert a coefficient at a time and return
  ///        the key for the coefficient
  ///
  /// If a coefficient with this time already exists, it is overwritten
  Key insertCoefficient(Time time, const Coefficient& coefficient);

  /// \brief Insert coefficients. Optionally returns the keys for these coefficients.
  ///
  /// If outKeys is not NULL, this function will not check if
  /// it is empty; new keys will be appended to this vector.
  void insertCoefficients(const std::vector<Time>& times, const std::vector<Coefficient>& values, std::vector<Key>* outKeys = nullptr);

  /// \brief Efficient function for adding a coefficient at the end of the map
  void addCoefficientAtEnd(Time time, const Coefficient& coefficient, std::vector<Key>* outKeys = nullptr);

  /// \brief Modify a coefficient by specifying a new time and value
  void modifyCoefficient(typename TimeToKeyCoefficientMap::iterator it, Time time, const Coefficient& coefficient);

  /// \brief Remove the coefficient with this key.
  ///
  /// It is an error if the key does not exist.
  void removeCoefficientWithKey(Key key);

  /// \brief Remove the coefficient at this time
  ///
  /// It is an error if there is no coefficient at this time.
  void removeCoefficientAtTime(Time time);

  /// \brief return true if there is a coefficient at this time
  bool hasCoefficientAtTime(Time time) const;

  /// \brief return true if there is a coefficient with this key
  bool hasCoefficientWithKey(Key key) const;

  /// \brief set the coefficient associated with this key
  ///
  /// This function fails if there is no coefficient associated
  /// with this key.
  void updateCoefficientByKey(Key key, const Coefficient& coefficient);

  /// \brief set the coefficient associated with this key

  /// \brief get the coefficient associated with this key
  Coefficient getCoefficientByKey(Key key) const;

  /// \brief get the coefficient time associated with this key
  Time getCoefficientTimeByKey(Key key) const;

  /// \brief Get the coefficients that are active at a certain time.
  ///
  /// This method can fail if the time is out of bounds. If it
  /// Succeeds, the elements of the pair are guaranteed to be filled
  /// nonnull.
  ///
  /// @returns true if it was successful
  bool getCoefficientsAt(Time time, CoefficientIter* outCoefficient0, CoefficientIter* outCoefficient1) const;

  /// \brief Get the coefficients that are active within a range \f$[t_s,t_e) \f$.
  void getCoefficientsInRange(Time startTime, Time endTime, CoefficientMap* outCoefficients) const;

  /// \brief Get all of the curve's coefficients.
  void getCoefficients(CoefficientMap* outCoefficients) const;

  /// \brief Set coefficients.
  ///
  /// If any of these coefficients doen't exist, there is an error
  void updateCoefficients(const CoefficientMap& coefficients);

  /// \brief return the number of coefficients
  size_t size() const;

  /// \brief Check if the manager is empty.
  bool empty() const;

  /// \brief clear the coefficients
  void clear();

  /// The first valid time for the curve.
  Time getMinTime() const;

  /// The one past the last valid time for the curve.
  Time getMaxTime() const;

  CoefficientIter coefficientBegin() const { return timeToCoefficient_.begin(); }

  CoefficientIter coefficientEnd() const { return timeToCoefficient_.end(); }

  typename TimeToKeyCoefficientMap::iterator coefficientBegin() { return timeToCoefficient_.begin(); }

  typename TimeToKeyCoefficientMap::iterator coefficientEnd() { return timeToCoefficient_.end(); }

  /// Check the internal consistency of the data structure
  /// If doExit is true, the function will call exit(0) at
  /// the end. This is useful for gtest death tests
  void checkInternalConsistency(bool doExit = false) const;

 private:
  /// Key to coefficient mapping
  boost::unordered_map<Key, typename TimeToKeyCoefficientMap::iterator> keyToCoefficient_;

  /// Time to coefficient mapping
  TimeToKeyCoefficientMap timeToCoefficient_;

  bool hasCoefficientAtTime(Time time, CoefficientIter* it, double tol = 0);
};

}  // namespace curves

#include "LocalSupport2CoefficientManager-inl.hpp"