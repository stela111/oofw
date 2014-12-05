#ifndef GANTRY_H
#define GANTRY_H

class Planner;

/// Controls the printer mechanics in machine coordinates.
/**
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

  /// Push move to planner.
  /** As a Gantry move may consist of many Planner moves,
      this method needs to be called repeatedly until it
      returns false.
      @returns true until move completely pushed to planner
      @note If planner is full, no move is pushed and false is returned.
      @note The same planner needs to be used for all push() calls
  */
  virtual bool push(Planner &planner) = 0;
};

#endif
