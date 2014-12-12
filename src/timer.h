#ifndef TIMER_H
#define TIMER_H

#include <cstdint>

class TimerCallback {
 public:
  /// Return requested delay to next call
  /** counting from start of this call.
      If 0 is returned, timer stops.
      The delay value is calculated as requested delay in seconds
      multiplied with frequency() for the associated Timer.
   */  
  virtual std::uint32_t on_timer() = 0;
};

/// Provides asynchronous timer callbacks at requested points in time
/** Callbacks on objects implementing
    the TimerCallback interface.
 */
class Timer {
 public:
  /// Start calling cb until it returns 0, see TimerCallback.
  virtual void start(TimerCallback *cb) = 0;
  
  /// Stop calling cb
  virtual void stop() = 0;

  /// The frequency in Hz for timer delay counter.
  virtual float frequency() const = 0;
};

#endif
