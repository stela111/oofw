#include "trapezoid_ticker.h"
#include "stepper.h"
#include "planner.h"
#include <cmath>

TrapezoidTicker::TrapezoidTicker(const std::vector<Stepper *>& steppers,
				 Timer* timer)
  : timer(timer)
  , steppers(steppers)
  , bresenhams(steppers.size())
  , unstep(false)
  , step_duration(static_cast<uint32_t>(10e-6f * timer->frequency()))
{}


void TrapezoidTicker::start(Planner *move_provider)
{
  this->move_provider = move_provider;
  timer->start(this);
}

void TrapezoidTicker::setup_next_move() {
  const Move *move = move_provider->get_current_move();
  if (move) {
    float rate = 1e3f;
    float events_per_mm = rate / move->speed;
    unsigned events = static_cast<unsigned>(events_per_mm * move->length);
    float entry_speed = std::sqrt(move_provider->get_current_entry_speed_sqr());
    float exit_speed = std::sqrt(move_provider->get_current_exit_speed_sqr());

    for (unsigned ind = 0; ind < steppers.size(); ind++) {
      steppers[ind]->set_direction(move->steps[ind]>0);
    }

    TrapezoidParameters params(events,
			       entry_speed * events_per_mm,
			       exit_speed * events_per_mm,
			       rate, /* = speed * events_per_mm */
			       timer->frequency(),
			       move->acceleration * events_per_mm);
    trapezoid = TrapezoidGenerator(params);
    
    for (unsigned ind = 0; ind < steppers.size(); ind++) {
      bresenhams[ind] = Bresenham(std::abs(move->steps[ind]), events, 1);
    }
    move_provider->next_move();
  }
}

std::uint32_t TrapezoidTicker::on_timer() {
  std::uint32_t next_delay = 0; // This means: stop timer
  
  if (unstep) {
    unstep = false;

    for (unsigned stepper = 0; stepper < steppers.size(); ++stepper) {
      steppers[stepper]->unstep();
    }
    next_delay = trapezoid.next_delay() - step_duration;

    if (trapezoid.is_done()) {
      // Try to setup next move now
      setup_next_move();
    }
  }
  else {
    // If just started, we need to start by setting up trapezoid.
    if (trapezoid.is_done()) {
      setup_next_move();
      // Todo: Reset timer to avoid getting next step too quickly
    }

    if (!trapezoid.is_done()) {
      for (unsigned stepper = 0; stepper < steppers.size(); ++stepper) {
	if (bresenhams[stepper].tick()) {
	  steppers[stepper]->step();
	}
      }
      unstep = true;
      next_delay = step_duration;
    }
  }

  return next_delay;
}
