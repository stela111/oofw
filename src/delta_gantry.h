#ifndef DELTA_GANTRY
#define DELTA_GANTRY

#include <vector>
#include "gantry.h"

class Planner;

/** Gantry implementation for delta geometry
 */
class DeltaGantry : public Gantry {
 public:
  /// Per axis configuration
  struct Axis {
    float steps_per_mm;
    float min_time_per_step;
    float min_time2_per_step;
  };

  /// Delta tower configuration.
  /** All towers move parallel with z-axis.
      The end effector is regarded to have no size,
      which means that the tower origin is defined
      as the x,y offset between arm joints when
      end effector is positioned on center of bed.
      The z offset is used to compensate for end stop position.
   */
  struct Tower {
    float origin[3];
    float arm_length;
  };

  DeltaGantry(const std::vector<Axis>& axes, const std::vector<Tower>& towers);

  // Gantry interface

  void set_cartesian(unsigned index, float pos);
  void set_extruder(unsigned index, float pos);
  void set_speed(float speed);
  bool get_move(LinearMove &move);

 private:
  /// Update next to position at most max_length towards target.
  /** As a side-effect, updates length member if returned true.
      @returns false if less than min_length from target, else true.
   */
  bool update_next_pos();

  /// Update next.steps from cartesian and extruder_pos
  void update_next_steps();

  /// Calculate steps for all axes
  void update_steps(std::vector<int>& steps);

  /// Calculate length on each cartesian axis relative vector length
  void update_next_unit_direction();

  // Returns max of cartesian vector distance and length of extruder moves
  float get_move_length() const;

  /// Return min time limited by each axis and steps for current move
  float min_time(const std::vector<int>& steps) const;

  /// Return min time2 limited by each axis and steps for current move
  float min_time2(const std::vector<int>& steps) const;

  /// Return max entry speed.
  /** Calculated by centripetal acceleration assuming
      moves are joined by circular path deviating junction_deviation from
      corner.
  */
  float max_entry_speed(float acc) const;

  /// Delta towers controlling the gantry position
  std::vector<Tower> towers;

  /// All axes, the first corresponding to towers,
  /// the remaining controlling the extruders
  std::vector<Axis> axes;

  struct Move {
    float cartesian[3];
    float unit_direction[3];
    std::vector<float> extruder_pos;
    std::vector<int> steps;
  };

  Move last, next;
  float target_cartesian[3];
  std::vector<float> target_extruder_pos;

  float length;
  float junction_deviation;
  float requested_speed;
  float requested_acc;
  float max_length;
  float min_length;
};


#endif
