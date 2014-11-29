#ifndef TIMER_H
#define TIMER_H

#include <cstdint>

class TimerCallback {
 public:
  virtual void on_timer() = 0;
};

/// Provides asynchronous timer callbacks at requested points in time
/** Callbacks on objects implementing
    the TimerCallback interface.
 */
class Timer {
 public:
  virtual void reset_time() = 0;
  virtual void call_at(std::uint32_t at, TimerCallback *cb) = 0;
};

#endif
