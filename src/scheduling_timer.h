#ifndef SCHEDULING_TIMER
#define SCHEDULING_TIMER

#include <stdint.h>

/// 
/**
   
 */
class Schedulable {
 protected:
  Schedulable()
    : timestamp(0)
    , next(nullptr)
  {
  }

  virtual ~Schedulable() {
  }

  /// Called as scheduled, see SchedulingTimer.
  virtual void on_timer() = 0;

  /// Return last scheduled timestamp
  uint16_t get_timestamp() const {
    return timestamp;
  }
 private:
  template <class Base>
  friend class SchedulingTimer;

  uint16_t timestamp;
  Schedulable *next;
};

/**
   Interface for timer interrupt callback
 */
struct TimerCallback {
  virtual void on_timer() = 0;
};

/**
 * Scheduling timer algorithm, calls Schedulables on requested timestamps.
 *
 * template parameter TimerBase must implement the following interface:
 *
 * \code
// Set callback for timer interrupt
void set_callback(TimerCallback *cb);

// Frequency of timer ticks in Hertz
uint32_t frequency();

// Return current timestamp
uint16_t current_timestamp();

// Start timer, callback on timestamp
void start_timer(uint16_t timestamp);

// Stop timer and clear time
void stop_timer();

// Disable all interrupts
void disable_interrupts();

// Enable all interrupts
void enable_interrupts();


\code
 * 
 */
template <class TimerBase>
class SchedulingTimer : public TimerCallback {
 public:
  
  SchedulingTimer()
    : head(nullptr)
  {
    base.set_callback(this);
  }

  uint32_t frequency() const {
    return base.frequency();
  }

  /**
   * Schedule block to run at timestamp
   */
  void schedule(Schedulable *sched, uint16_t timestamp) {
    base.disable_interrupts();

    Schedulable *prev = nullptr;
    Schedulable *current = head;
    while(current && static_cast<int32_t>(timestamp - current->timestamp) > 0)
    {
      prev = current;
      current = current->next;
    }

    if (!prev) {
      head = sched;
      base.start_timer(timestamp);
    }
    else {
      prev->next = sched;
    }

    sched->next = current;
    sched->timestamp = timestamp;

    base.enable_interrupts();
  }


  void remove(Schedulable *sched) {
    base.disable_interrupts();
    if (head == sched) {
      head = sched->next;
      base.start_timer(head->timestamp);
    }
    else {
      Schedulable *last = head;
      Schedulable *current = head->next;
      while(current) {
	if (current == sched) {
	  last->next = sched->next;
	  break;
	}
	last = current;
	current = current->next;
      }
    }
    base.enable_interrupts();
  }

  void on_timer() {
    while(true) {
      if (!head) {
	base.stop_timer();
	return;
      }
      
      if (static_cast<int32_t>(head->timestamp - 
			       base.current_timestamp()) <= 0) {
	Schedulable *current = head;
	head = head->next;
	current->on_timer();
      }
      else {
	base.start_timer(head->timestamp);
	return;
      }
    }
  }

  /// Provide access to TimerBase
  TimerBase &get_base() {
    return base;
  }
 private:
  TimerBase base;
  Schedulable *head;
};

#endif
