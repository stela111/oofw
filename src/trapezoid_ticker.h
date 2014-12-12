#ifndef TRAPEZOID_TICKER_H
#define TRAPEZOID_TICKER_H

#include <vector>
#include "timer.h"
#include "bresenham.h"
#include "trapezoid_generator.h"

class Stepper;
class Planner;

/// Generate steps following trapezoid profile.
/**
   This class uses a Timer to generate a time base with a trapezoid shaped
   frequency, and uses this to generate step pulses to stepper motors. The
   trapzeoid and step pulses are supplied by a MoveProvider.
 */
class TrapezoidTicker : public TimerCallback {
 public:
  TrapezoidTicker(const std::vector<Stepper*>& steppers, Timer *timer);

  /// Start generating steps until no more moves are available.
  /** Moves are repeatedly pulled from the move_provider.
      @note The calls to move_provider are executed from interrupt context
  */
  void start(Planner *move_provider);

 private:
  void setup_next_move();
  std::uint32_t on_timer();
  Timer *timer;
  std::vector<Stepper*> steppers;
  std::vector<Bresenham> bresenhams;
  Planner *move_provider;
  TrapezoidGenerator trapezoid;
  bool unstep;
  std::uint32_t step_duration;
};

#endif
