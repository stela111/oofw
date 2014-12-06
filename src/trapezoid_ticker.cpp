#include "trapezoid_ticker.h"
#include "stepper.h"

TrapezoidTicker::TrapezoidTicker(const std::vector<Stepper *>& steppers,
				 Timer* timer)
  : timer(timer)
  , steppers(steppers)
  , bresenhams(steppers.size())
{}


void TrapezoidTicker::generate(const std::vector<int>& steps,
			       unsigned events,
			       float entryRate,
			       float exitRate,
			       float cruiseRate,
			       float acc)
{
  TrapezoidParameters params(events, entryRate, exitRate, cruiseRate, 1e6, acc);
  trapezoid = TrapezoidGenerator(params);

  for (unsigned stepper = 0; stepper < steppers.size(); ++stepper) {
    bresenhams[stepper] = Bresenham(steps[stepper], events);
  }

  timer->start(this);
}


std::uint32_t TrapezoidTicker::on_timer() {
  for (unsigned stepper = 0; stepper < steppers.size(); ++stepper) {
    if (bresenhams[stepper].tick()) {
      steppers[stepper]->step();
    }
  }

  std::uint32_t next = trapezoid.next_delay();

  for (unsigned stepper = 0; stepper < steppers.size(); ++stepper) {
    steppers[stepper]->unstep();
  }

  return next;
}
