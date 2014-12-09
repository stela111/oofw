#ifndef TRAPEZOID_TICKER_H
#define TRAPEZOID_TICKER_H

#include <vector>
#include "timer.h"
#include "bresenham.h"
#include "trapezoid_generator.h"

class Stepper;

class TrapezoidTicker : public TimerCallback {
 public:
  TrapezoidTicker(const std::vector<Stepper*>& steppers, Timer *timer);

  /// Generate steps following trapezoid profile.
  /** Note: Trapezoid may be degenerate such that cruise speed is never
      reached. But it must be possible to reach final speed within count
      given initial speed and acceleration.
  */
  void generate(const std::vector<int> &steps,
		unsigned events,
		float entryRate,
		float exitRate,
		float cruiseRate,
		float acc);

 private:
  std::uint32_t on_timer();
  Timer *timer;
  std::vector<Stepper*> steppers;
  std::vector<Bresenham> bresenhams;
  TrapezoidGenerator trapezoid;
  bool unstep;
  std::uint32_t step_duration;
};

#endif
