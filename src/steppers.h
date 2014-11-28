#ifndef STEPPERS_H
#define STEPPERS_H

/**
   @brief Performs synchronized accelerated moves of several Stepper motors.

   Maintains a queue of moves that are performed as quickly as possible
   while respecting speed, acc and jerk constraints on each Stepper.

   example homing procedure:
   
       while(steppers.state() == RUNNING);

       steppers.stop_on_endstop(true);
       steppers.set_position(home);
       steppers.queue_move(beyond_endstops, safe_speed);
     
       while(steppers.state() == RUNNING);

       if (steppers.state() == IDLE) {
         homing_failed();
       }
       else {
         steppers.set_position(home);
       }
*/
class Steppers {
 public:
  /** Describes the state of the moves performed by Steppers
   */
  enum class State {
    IDLE, ///< All motors stopped and queue empty
    RUNNING, ///< Performing move
    STOPPED_ON_ENDSTOP ///< Last move ended on triggered endstop
  };

  /// Stepper positions in millimeters
  struct Positions {
    float x,y,z,e;
  };

  /// Return true if there is space for a move in the queue
  bool queue_not_full() const;

  /// Queue a movement
  /** @param pos target Stepper positions at end of move
      @param speed max speed during this move
      @pre Only allowed when queue_not_full() == true
   */
  void queue_move(Positions pos, float speed);

  /// Queue a movement with additional limits on entry speed and acceleration.
  /**
     @param pos target Stepper positions at end of move
     @param speed max speed during this move
     @param entry_speed max speed at start of this move
     @param acc max acceleration for this move
     @pre Only allowed when queue_not_full() == true
   */
  void queue_move(Positions pos, float speed, float entry_speed, float acc);

  /// Returns current state
  State state() const;

  /// Set current position.
  /** @pre For deterministic behaviour, call only in State() != RUNNING
  */
  void set_position(Positions pos);

  /// Get current position
  Positions get_position() const;

  /// Set behaviour on triggered endstop.
  /** If argument is true, a triggered endstop stops all
      steppers and clears queue. State is set to STOPPED_ON_ENDSTOP.
      @pre For deterministic behaviour, call only in State() != RUNNING
  */
  void stop_on_endstop(bool);
};

#endif
