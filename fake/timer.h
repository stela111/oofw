#ifndef TIMER_FAKE
#define TIMER_FAKE

#include <src/timer.h>

namespace fake {

  class Timer : public ::Timer {
  public:
    void start(TimerCallback *cb) {
      callback = cb;
    }
    
    void stop() {
    }
  
    std::uint32_t fake_next() {
      if (callback) {
	std::uint32_t delay = callback->on_timer();
	if (delay == 0) {
	  callback = 0;
	}
	return delay;
      }
      return 0;
    }
    
  private:
    TimerCallback *callback;
  };
  
}

#endif
