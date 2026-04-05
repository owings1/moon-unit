#ifndef MOPERF_H
#define MOPERF_H
#include <Arduino.h>
class JitterMonitor {
public:
  uint32_t count = 0;
  double mean = 0.0;
  double m2 = 0.0;
  uint32_t max_val = 0;
  volatile bool locked = false;

  // Call this every loop - No sqrt, minimal division
  void observe(uint32_t value) {
    locked = true;
    count++;
    auto delta = value - mean;
    mean += delta / count;
    auto delta2 = value - mean;
    m2 += delta * delta2;
    if (value > max_val) max_val = value;
    locked = false;
  }

  // Call this only when Python requests the data
  float get_stdev() const {
    if (count < 2) return 0.0f;
    return (float)sqrt(m2 / (count - 1));
  }

  void reset() {
    count = 0;
    mean = 0.0;
    m2 = 0.0;
    max_val = 0;
  }
};
#endif