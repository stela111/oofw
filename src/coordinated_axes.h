#ifndef COORDINATED_AXES_H
#define COORDINATED_AXES_H

#include <cstdint>

#include "timer.h"
#include "stepper.h"
#include "planner.h"

/**
   @brief Performs synchronized accelerated moves of several axes
   controlled by Stepper motors.

   Maintains a queue of moves that are performed as quickly as possible
   while respecting speed, acc and jerk constraints on each Stepper.
*/
class CoordinatedAxes : TimerCallback {
 public:
  /// Create a CoordinatedAxes object
  CoordinatedAxes(Timer *timer);

  /// Add an axis
  unsigned add_axis(Stepper *stepper);
  
  /// Set target axis position for next move
  void plan_target(unsigned axis, float position);

  /// Describes the state of the moves performed by Steppers
  enum class State {
    IDLE, ///< All motors stopped and queue empty
    RUNNING, ///< Performing move
    STOPPED_ON_ENDSTOP ///< Last move ended on triggered endstop
  };


  /// Return true if there is space for a move in the queue
  bool queue_not_full() const;

  /// Queue a movement to target set by plan_target()
  /** @param speed max speed during this move
      @pre Only allowed when queue_not_full() == true
   */
  void queue_move(std::uint32_t us);

  /// Queue a movement with additional limits on entry speed and acceleration.
  /**
     @param speed max speed during this move
     @param entry_speed max speed at start of this move
     @param acc max acceleration for this move
     @pre Only allowed when queue_not_full() == true
   */
  void queue_move(float speed, float entry_speed, float acc);

  /// Returns current state
  State state() const {
    return state_;
  }

  /// Set current position.
  /** @pre For deterministic behaviour, call only in State() != RUNNING
  */
  void set_position(unsigned axis, float pos);

  /// Get current position
  float get_position(unsigned axis) const;

  /// Set behaviour on triggered endstop.
  /** If argument is true, a triggered endstop stops all
      steppers and clears queue. State is set to STOPPED_ON_ENDSTOP.
      @pre For deterministic behaviour, call only in State() != RUNNING
  */
  void stop_on_endstop(bool);

  /// TimerCallback interface
  void on_timer();

 private:
  Timer *timer_;
  State state_;
  Planner planner_;

  struct Axis {
    Stepper *stepper;
    std::int32_t position;
    std::int32_t next_steps;
  };
  
  std::vector<Axis> axes_;
};

#endif
