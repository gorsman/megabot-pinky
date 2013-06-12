#ifndef _MegaBot_profiling_h_
#define _MegaBot_profiling_h_

namespace profiling {

// Utility class that helps to measure time between consecutive events.
//
// Usage example:
//   // Initialization: declare Ticker instance to be reused on multiple
//   // occasions.
//   Ticker local_ticker;
//   ...
//   // Measuring start: remember current time.
//   local_ticker.Tick();
//   ...  // Code for which we're measuring execution time.
//   // Measuring end: recording the time it took to execute the code.
//   long execution_time_micros = local_ticker.Tick();
//
class Ticker {
  public:
    Ticker() : last_tick_(0L) { Tick(); }
    
    // Returns the number of microseconds that have passed since
    // last Tick call.
    long Tick();

    static Ticker shared_ticker;

  protected:
    long last_tick_;
};

// For convenience we define a TICK() macro that uses a shared Ticker
// instance. That way you don't need to define a Ticker instance of your own.
#define TICK() ::profiling::Ticker::shared_ticker.Tick()


// Utility class to aggregate basic stats (avg/min/max) for a set of values.
class ValueStat {
  public:
    ValueStat();
    void Clear();
    void Record(long val);

  public:
    long count, sum, min, max;
};

#define VALUE_STAT_PRINT(msg, stat) { \
  if (stat.count == 0) { \
    DEBUG_PRINT2(msg, "n/a"); \
    return; \
  } \
  long avg = stat.sum / stat.count; \
  DEBUG_PRINT7(msg, "avg:", avg, "min:", stat.min, "max:", stat.max); \
}


}  // namespace profiling

#endif  // #ifndef _MegaBot_profiling_h_
