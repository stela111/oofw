#ifndef STEPPER_H
#define STEPPER_H

#include <cstdint>
#include "pin_io.h"

/** Stepper is a light-weight control of i/o for one stepper motor.
    Keeps track of position in steps.
    Knows how long a step is and physical limits of motion.
 */
class Stepper {
 public:
  /// Mapping to i/o pins
  struct Pins {
    PinIo::Pin enable;
    PinIo::Pin step;
    PinIo::Pin dir;
    PinIo::Pin endstop;
  };

  Stepper(PinIo *io, Pins pins);
  
  /// State of stepper motor
  enum class State {
    DISABLED, ///< Motor power is off
    ACTIVE,   ///< Motor power is on, reacting to steps
    STEPPING, ///< Between step() and unstep()
    STOPPED,  ///< Motor power is on, not reacting to steps
  };

  /// Return current state
  State state() const;

  /// Disable motor (power off)
  void disable();

  /// Disable motor (power on)
  void enable();

  /// Set direction
  /**
      @param positive if true, step() increases position by one,
      otherwise decreases by one.
   */
  void set_direction(bool positive);

  /// Do one step if in state ACTIVE
  void step();
  
  /// Release step. Needs to be done at least XX us after step().
  void unstep();

  /// Check if endstop is currently active
  bool is_endstop_active() const;

  /// Select behaviour on endstop.
  /** If true, when endstop is triggered during step,
      state is set to STOPPED.
      If false, ignore endstop.
   */
  void stop_on_endstop(bool);

  /// Set current position
  void set_position(int position) {
    position_ = position;
  }

  /// Get current position
  int position() const {
    return position_;
  }

 private:
  PinIo *io_;
  Pins pins_;

  std::int32_t position_;
  State state_;
  bool direction_;
  bool stop_on_endstop_;
};

#endif
