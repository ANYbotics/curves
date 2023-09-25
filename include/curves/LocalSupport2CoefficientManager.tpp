#include "curves/LocalSupport2CoefficientManager.hpp"

#include <iostream>

#include <message_logger/message_logger.hpp>

#include "curves/KeyGenerator.hpp"

namespace curves {

/// Compare this Coefficient manager with another for equality.
template <class Coefficient>
bool LocalSupport2CoefficientManager<Coefficient>::equals(const LocalSupport2CoefficientManager& other) const {
  bool equal = true;
  equal &= keyToCoefficient_.size() == other.keyToCoefficient_.size();
  equal &= timeToCoefficient_.size() == other.timeToCoefficient_.size();
  if (equal) {
    CoefficientIter it1;
    CoefficientIter it2;
    it1 = keyToCoefficient_.begin();
    it2 = other.keyToCoefficient_.begin();
    for (; it1 != keyToCoefficient_.end(); ++it1, ++it2) {
      equal &= it1->first == it2->first;
      equal &= it1->second.equals(it2->second);
    }

    CoefficientIter it3;
    CoefficientIter it4;
    it3 = timeToCoefficient_.begin();
    it4 = other.timeToCoefficient_.begin();
    for (; it3 != timeToCoefficient_.end(); ++it3, ++it4) {
      equal &= it3->first == it4->first;
      if (it3->second && it4->second) {
        equal &= it3->second->equals(*(it4->second));
      } else {
        equal &= it3->second == it4->second;
      }
    }
  }
  return equal;
}

template <class Coefficient>
void LocalSupport2CoefficientManager<Coefficient>::getTimes(std::vector<Time>* outTimes) const {
  assert(outTimes != nullptr);
  outTimes->clear();
  outTimes->reserve(timeToCoefficient_.size());
  auto it = timeToCoefficient_.begin();
  for (; it != timeToCoefficient_.end(); ++it) {
    outTimes->push_back(it->first);
  }
}

template <class Coefficient>
Key LocalSupport2CoefficientManager<Coefficient>::insertCoefficient(Time time, const Coefficient& coefficient) {
  CoefficientIter it;
  Key key{};

  if (this->hasCoefficientAtTime(time, &it)) {
    this->updateCoefficientByKey(it->second.key_, coefficient);
    key = it->second.key_;
  } else {
    key = KeyGenerator::getNextKey();
    std::pair<Time, KeyCoefficient> iterator(time, KeyCoefficient(key, coefficient));
    auto success = timeToCoefficient_.insert(iterator);
    keyToCoefficient_[key] = success.first;
  }
  return key;
}

/// \brief insert coefficients. Optionally returns the keys for these coefficients
template <class Coefficient>
void LocalSupport2CoefficientManager<Coefficient>::insertCoefficients(const std::vector<Time>& times,
                                                                      const std::vector<Coefficient>& values, std::vector<Key>* outKeys) {
  assert(times.size() == values.size());
  for (Key i = 0; i < times.size(); ++i) {
    if (outKeys != nullptr) {
      outKeys->push_back(insertCoefficient(times[i], values[i]));
    } else {
      insertCoefficient(times[i], values[i]);
    }
  }
}

template <class Coefficient>
void LocalSupport2CoefficientManager<Coefficient>::addCoefficientAtEnd(Time time, const Coefficient& coefficient,
                                                                       std::vector<Key>* outKeys) {
  assert(time > getMaxTime() && "Time to add is not greater than curve max time.");

  Key key = KeyGenerator::getNextKey();

  // Insert the coefficient with a hint that it goes at the end
  auto it = timeToCoefficient_.insert(--(timeToCoefficient_.end()), std::make_pair(time, KeyCoefficient(key, coefficient)));

  keyToCoefficient_.insert(keyToCoefficient_.end(), std::make_pair(key, it));
  if (outKeys != nullptr) {
    outKeys->push_back(key);
  }
}

template <class Coefficient>
void LocalSupport2CoefficientManager<Coefficient>::modifyCoefficient(typename TimeToKeyCoefficientMap::iterator it, Time time,
                                                                     const Coefficient& coefficient) {
  // This is used by slerp sampling policy.
  // In this case a new coefficient should be placed slightly later than the initial one.
  auto newIt = timeToCoefficient_.insert(it, std::make_pair(time, KeyCoefficient(it->second.key_, coefficient)));
  // Update keyToCoefficient_
  keyToCoefficient_[it->second.key_] = newIt;
  // Remove the old coefficient
  timeToCoefficient_.erase(it);
}

template <class Coefficient>
void LocalSupport2CoefficientManager<Coefficient>::removeCoefficientAtTime(Time time) {
  assert(this->hasCoefficientAtTime(time) && "No coefficient at that time.");
  auto it1 = timeToCoefficient_.find(time);
  auto it2 = keyToCoefficient_.find(it1->second.key_);
  timeToCoefficient_.erase(it1);
  keyToCoefficient_.erase(it2);
}

/// \brief return true if there is a coefficient at this time
template <class Coefficient>
bool LocalSupport2CoefficientManager<Coefficient>::hasCoefficientAtTime(Time time) const {
  auto it = timeToCoefficient_.find(time);
  return it != timeToCoefficient_.end();
}

/// \brief set the coefficient associated with this key
///
/// This function fails if there is no coefficient associated
/// with this key.
template <class Coefficient>
void LocalSupport2CoefficientManager<Coefficient>::updateCoefficientByKey(Key key, const Coefficient& coefficient) {
  auto it = keyToCoefficient_.find(key);
  assert(it != keyToCoefficient_.end() && "Key is not in the container.");
  it->second->second.coefficient_ = coefficient;
}

/// \brief Get the coefficients that are active at a certain time.
template <class Coefficient>
bool LocalSupport2CoefficientManager<Coefficient>::getCoefficientsAt(Time time, CoefficientIter* outCoefficient0,
                                                                     CoefficientIter* outCoefficient1) const {
  assert(outCoefficient0 != nullptr);
  assert(outCoefficient1 != nullptr);
  if (timeToCoefficient_.empty()) {
    MELO_INFO_STREAM("No coefficients");
    return false;
  }

  if (time < getMinTime() || time > getMaxTime()) {
    MELO_INFO_STREAM("time, " << time << ", is out of bounds: [" << getMinTime() << ", " << getMaxTime() << "]");
    return false;
  }

  auto it = timeToCoefficient_.upper_bound(time);

  // Check for edge cases
  if (it == timeToCoefficient_.end()) {
    --it;
  } else if (it == timeToCoefficient_.begin()) {
    ++it;
  }

  *outCoefficient1 = it--;
  *outCoefficient0 = it;

  return true;
}

/// \brief return the number of coefficients
template <class Coefficient>
Key LocalSupport2CoefficientManager<Coefficient>::size() const {
  return timeToCoefficient_.size();
}

template <class Coefficient>
bool LocalSupport2CoefficientManager<Coefficient>::empty() const {
  return timeToCoefficient_.empty();
}

/// \brief clear the coefficients
template <class Coefficient>
void LocalSupport2CoefficientManager<Coefficient>::clear() {
  keyToCoefficient_.clear();
  timeToCoefficient_.clear();
}

template <class Coefficient>
Time LocalSupport2CoefficientManager<Coefficient>::getMinTime() const {
  if (timeToCoefficient_.empty()) {
    return 0;
  }
  return timeToCoefficient_.begin()->first;
}

template <class Coefficient>
Time LocalSupport2CoefficientManager<Coefficient>::getMaxTime() const {
  if (timeToCoefficient_.empty()) {
    return 0;
  }
  return timeToCoefficient_.rbegin()->first;
}

template <class Coefficient>
bool LocalSupport2CoefficientManager<Coefficient>::hasCoefficientAtTime(Time time, CoefficientIter* it, double tol) {
  for ((*it) = timeToCoefficient_.begin(); (*it) != timeToCoefficient_.end(); ++(*it)) {
    if ((*it)->first >= time - tol) {
      if ((*it)->first <= time + tol) {
        return true;
      } else {
        return false;
      }
    }
  }
  return false;
}

}  // namespace curves
