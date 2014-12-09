#ifndef GANTRY_H
#define GANTRY_H

class Planner;

/// Controls the printer mechanics in machine coordinates.
/**
   This class is responsible for converting a request to move
   the gantry a certain distance into stepper motor steps for
   the axes used to control the gantry and extruders.
   Input is in units millimeters and seconds, whereas
   output is in units of steps and seconds.

   Lengthy operations are performed asynchronously. First,
   a move is set up, then it is pushed to a Planner.
   A Gantry move may consist of many Planner moves. Therefore
   pushing the move to a planner may need to be performed several
   times before setting up a new Gantry move.
   This design is done to keep each operation as short as possible.
*/
class Gantry {
 public:
  virtual ~Gantry() {}

  /// Set target position for gantry.
  /** @param index is 0-2 corresponding to x,y,z
      @param pos is target position in millimeters
  */
  virtual void set_cartesian(unsigned index, float pos) = 0;

  /// Set target position for extruder
  /** @param index is 0-based extruder index
      @param pos is target position in millimeters
  */
  virtual void set_extruder(unsigned index, float pos) = 0;

  /// Set speed in mm/s
  virtual void set_speed(float speed) = 0;

  /// Specification for a linear stepper motion
  struct LinearMove {
    /// Stepper steps for each axis in the Gantry.
    std::vector<int> steps;
    float cruise_speed;
    float acceleration;
    float entry_speed;
    float length;
  };

  /// Get linear move.
  /** As a Gantry move may consist of many Planner moves,
      this method needs to be called repeatedly until it
      returns false.
      @param move set to next linear segment
      @returns false when no more moves are needed.
               @a move has not been modified.
  */
  bool get_move(LinearMove& move);
};

#endif
