#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#include "src/scheduling_timer.h"

extern "C" void __cxa_pure_virtual(void) {}

void operator delete(void * ptr)
{
  //  free(ptr);
}

/*
void * operator new(size_t size)
{
  //  return malloc(size);
}


void * operator new[](size_t size) 
{ 
  //    return malloc(size); 
} 

void operator delete[](void * ptr) 
{ 
  //    free(ptr); 
} 
*/

class Timer1 {
public:
  Timer1() {
    TCCR1A = 0;
    TCNT1 = 0;
    TCCR1B = _BV(CS11) | _BV(CS10) ; // F_CPU / 64
  }

  void set_callback(TimerCallback *cb) {
    Timer1::cb = cb;
  }

  ~Timer1() {
    stop_timer();
    TCCR1B = 0; // Stop timer clock
  }

  uint32_t frequency() const {
    return F_CPU/64;
  }
  void enable_interrupts() {
    sei();
  }
  void disable_interrupts() {
    cli();
  }
  void start_timer(uint16_t tick) {
    OCR2A = tick;
    TIMSK1 = _BV(OCIE1A);
  }
  void stop_timer() {
    TIMSK1 = 0; // Clear interrupt mask
    TCNT1 = 0; // Reset timestamp
  }
  uint16_t current_timestamp() {
    return TCNT1;
  }

  static void isr() {
    if (cb != nullptr) {
      cb->on_timer();
    }
  }
private:
  static TimerCallback *cb;
};

TimerCallback *Timer1::cb = 0;

ISR(TIMER1_COMPA_vect) {
  Timer1::isr();
}


struct Task1 : public Schedulable
{
  inline void on_timer() {
    PORTA |= _BV(PINA0);
  }
};

int main() {
  SchedulingTimer<Timer1> timer;
  
  Task1 t;
  timer.schedule(&t, timer.frequency()/100);

  return 0;
}
