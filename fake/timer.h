#ifndef TIMER_FAKE
#define TIMER_FAKE

#include <algorithm>
#include <list>
#include <src/timer.h>

namespace fake {

  class Timer : public ::Timer {
  public:
    void reset_time() {
      clock_ = 0;
    }
  
    void call_at(std::uint32_t at, TimerCallback *cb) {
      Entry e;
      e.cb = cb;
      e.at = at;
      list_.push_back(e);
    }
    
    std::uint32_t fake_next() {
      Entry& front = list_.front();
      std::uint32_t diff = front.at - clock_;
      clock_ = front.at;
      front.cb->on_timer();
      return diff;
    }
    
  private:
    TimerCallback *callback_;
    std::uint32_t clock_;
    
    struct Entry {
      TimerCallback *cb;
      std::uint32_t at;
    };
    
    std::list<Entry> list_;
  };
  
}

#endif
