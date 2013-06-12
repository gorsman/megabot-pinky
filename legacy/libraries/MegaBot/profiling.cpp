#include <Arduino.h>
#include "profiling.h"

namespace profiling {

//  Ticker
// ********

Ticker Ticker::shared_ticker;

long Ticker::Tick() {
  long cur = micros();
  long delta = cur - last_tick_;
  last_tick_ = cur;
  return delta;
}


//  ValueStat
// ***********

ValueStat::ValueStat() {
  Clear();
}
    
void ValueStat::Clear() {
  count = 0L;
  sum = 0L;
  min = 0L;
  max = 0L;
}

void ValueStat::Record(long val) {
  if (count == 0) {
    min = val;
    max = val;
  } else {
    min = (val < min) ? val : min;
    max = (val > max) ? val : max;
  }
  sum += val;
  ++count;
}

}  // namespace profiling
