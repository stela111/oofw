#include "trapezoid_ticker.h"
#include "stepper.h"
#include <cmath>

TrapezoidTicker::TrapezoidTicker(const std::vector<Stepper *>& steppers,
				 Timer* timer)
  : timer(timer)
  , steppers(steppers)
  , bresenhams(steppers.size())
  , unstep(false)
  , step_duration(10)
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
    bresenhams[stepper] = Bresenham(std::abs(steps[stepper]), events);
  }

  timer->start(this);
}


std::uint32_t TrapezoidTicker::on_timer() {
  if (unstep) {
    unstep = false;

    for (unsigned stepper = 0; stepper < steppers.size(); ++stepper) {
      steppers[stepper]->unstep();
    }

    std::uint32_t remaining = trapezoid.next_delay() - step_duration;

    if (trapezoid.is_done()) {
      // @todo: prepare next trapezoid
    }
    return remaining;
  }
  else {
    if (trapezoid.is_done()) {
      return 0;
    }
    
    for (unsigned stepper = 0; stepper < steppers.size(); ++stepper) {
      if (bresenhams[stepper].tick()) {
	steppers[stepper]->step();
      }
    }

    unstep = true;
    return step_duration;
  }
}
