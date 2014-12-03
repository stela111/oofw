#ifndef DELTA_GANTRY
#define DELTA_GANTRY

#include <vector>
#include "planner.h"

class DeltaGantry {
 public:
  DeltaGantry(unsigned extruders);

  void move();

 private:
  void update_tower_steps();
  void update_next_unit_direction(float length);

  // Returns max of cartesian distance and length of extruder moves
  float get_move_length() const;
  float limit_speed(float requested_speed, float length) const;
  float limit_acc(float requested_acc, float length) const;
  float limit_jerk(float acc) const;

  struct Axis {
    float steps_per_mm;
    float min_time_per_step;
    float min_time2_per_step;
  };
  
  struct Tower : Axis {
    float origin[3];
    float arm_length;
  };

  /// Axes controlling the gantry position
  std::vector<Tower> towers;

  /// Axes controlling the extruders
  std::vector<Axis> extruders;

  struct Move {
    float cartesian[3];
    float unit_direction[3];
    std::vector<float> extruder_pos;
    std::vector<int> tower_steps;
    std::vector<int> extruder_steps;
  };

  Move last, next;

  std::vector<int> steps;
  float junction_deviation;
  float requested_speed;
  float requested_acc;

  Planner planner;
};


#endif
