#ifndef TIMER_H
#define TIMER_H

#include <cstdint>

class TimerCallback {
 public:
  /// Return requested milliseconds to next call
  /** counting from start of this call.
      If 0 is returned, timer stops.
   */  
  virtual std::uint32_t on_timer() = 0;
};

/// Provides asynchronous timer callbacks at requested points in time
/** Callbacks on objects implementing
    the TimerCallback interface.
 */
class Timer {
 public:
  /// Start 
  virtual void start(TimerCallback *cb) = 0;
  virtual void stop() = 0;
};

#endif
