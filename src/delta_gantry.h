#ifndef DELTA_GANTRY
#define DELTA_GANTRY

#include <vector>
#include "planner.h"

/** Gantry implementation for delta geometry
 */
class DeltaGantry {
 public:
  struct Axis {
    float steps_per_mm;
    float min_time_per_step;
    float min_time2_per_step;
  };
  
  struct Tower {
    float origin[3];
    float arm_length;
  };

  DeltaGantry(const std::vector<Axis>& axes, const std::vector<Tower>& towers);

  void set_cartesian(unsigned index, float pos);
  void set_extruder(unsigned index, float pos);
  void set_speed(float speed);
  void move();

 private:
  /// Calculate next.steps for towers and steps for all axes
  /// (next.steps for extruders has already been set by set_extruder()).
  void update_steps();

  /// Calculate length on each cartesian axis relative vector length
  void update_next_unit_direction(float length);

  // Returns max of cartesian vector distance and length of extruder moves
  float get_move_length() const;

  /// Return speed limited by each axis
  float limit_speed(float requested_speed, float length) const;

  /// Return acceleration limited by each axis
  float limit_acc(float requested_acc, float length) const;

  /// Return speed limited by cartesian jerk calculation
  float limit_jerk(float acc) const;

  /// Delta towers controlling the gantry position
  std::vector<Tower> towers;

  /// All axes, the first corresponding to towers,
  /// the remaining controlling the extruders
  std::vector<Axis> axes;

  struct Move {
    float cartesian[3];
    float unit_direction[3];
    std::vector<int> steps;
  };

  Move last, next;

  std::vector<int> steps;
  float junction_deviation;
  float requested_speed;
  float requested_acc;

  Planner planner;
};


#endif
