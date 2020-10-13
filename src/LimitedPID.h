#ifndef __ARC_RELATIVE_PID__
#define __ARC_RELATIVE_PID__

#include "PID.h"

// https://github.com/Arduino-CI/arduino_ci/issues/165
#ifdef max
#undef max
#ifdef __cplusplus
template <class T, class L>
auto max(const T &a, const L &b) -> decltype((b < a) ? b : a) {
  return (a < b) ? b : a;
}
#else
#define max(a, b)                                                              \
  ({                                                                           \
    __typeof__(a) _a = (a);                                                    \
    __typeof__(b) _b = (b);                                                    \
    _a > _b ? _a : _b;                                                         \
  })
#endif
#endif

#ifdef min
#undef min
#ifdef __cplusplus
template <class T, class L>
auto min(const T &a, const L &b) -> decltype((b < a) ? a : b) {
  return (a < b) ? a : b;
}
#else
#define min(a, b)                                                              \
  ({                                                                           \
    __typeof__(a) _a = (a);                                                    \
    __typeof__(b) _b = (b);                                                    \
    _a > _b ? _b : _a;                                                         \
  })
#endif
#endif

#include <algorithm>
#include <tuple>
#include <utility>

namespace arc {

template <typename V> class LimitedPID {
private:
protected:
  PID<V> &pid;
  std::pair<V, V> outputLimits;

public:
  LimitedPID(PID<V> &pid, std::pair<V, V> outputLimits);
  void setInput(const V &value);
  void setInput(const V &value, unsigned long t);
  V getOutput(const V &last) const;
  void setTarget(V target);
};

template <typename V>
LimitedPID<V>::LimitedPID(PID<V> &pid, std::pair<V, V> outputLimits)
    : pid(pid) {
  this->outputLimits = outputLimits;
}

template <typename V> void LimitedPID<V>::setInput(const V &value) {
  this->pid.setInput(value);
}

template <typename V>
void LimitedPID<V>::setInput(const V &value, const unsigned long t) {
  this->pid.setInput(value, t);
}

template <typename V> void LimitedPID<V>::setTarget(V value) {
  this->pid.setTarget(value);
}

template <typename V> V LimitedPID<V>::getOutput(const V &last) const {
  return std::max(std::min(last + this->pid.getOutput(), outputLimits.second),
                  outputLimits.first);
}

}; // namespace arc

#endif