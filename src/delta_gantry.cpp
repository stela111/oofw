#include "delta_gantry.h"
#include <math.h>

namespace {
  const unsigned num_towers = 3;
};

DeltaGantry::DeltaGantry(unsigned num_extruders)
  : towers(num_towers)
  , extruders(num_extruders)
  , steps(num_towers + num_extruders)
  , requested_speed(0)
  , requested_acc(1000)
  , planner(16, num_towers + num_extruders)
{
  last.tower_steps.resize(num_towers);
  last.extruder_steps.resize(num_extruders);
  last.extruder_pos.resize(num_extruders);
  next.tower_steps.resize(num_towers);
  next.extruder_steps.resize(num_extruders);
  next.extruder_pos.resize(num_extruders);
}

  
void DeltaGantry::move()
{
  update_tower_steps();
  float length = get_move_length();
  update_next_unit_direction(length);
  float speed = limit_speed(requested_speed, length);
  float acc = limit_acc(requested_acc, length);
  float init_speed = limit_jerk(acc);
  planner.plan_move(steps, 
		    2*acc*length,
		    speed*speed,
		    init_speed*init_speed);
}


void DeltaGantry::update_tower_steps()
{
  for (unsigned tower = 0; tower < towers.size(); ++tower) {
    float dist[3];
    for (unsigned coord = 0; coord < 3; coord++) {
      dist[coord] = next.cartesian[coord] - towers[tower].origin[coord];
    }
    float xy_dist2 = dist[0]*dist[0] + dist[1]*dist[1];
    float arm_length2 = towers[tower].arm_length*towers[tower].arm_length;
    bool out_of_range = xy_dist2 > arm_length2;
    float tower_z = dist[2] + out_of_range ? 0 : sqrtf(arm_length2 - xy_dist2);

    next.tower_steps[tower] = tower_z*towers[tower].steps_per_mm;
  }
}


float DeltaGantry::get_move_length() const
{
  float sqr_sum = 0;
  for (unsigned coord = 0; coord < 3; coord++) {
    float delta = next.cartesian[coord] - last.cartesian[coord];
    sqr_sum += delta*delta;
  }
  
  for (unsigned extr = 0; extr < next.extruder_pos.size(); ++extr)
  {
    float delta = next.extruder_pos[extr] - last.extruder_pos[extr];
    sqr_sum = std::max(sqr_sum, delta*delta);
  }

  return sqrtf(sqr_sum);
}


void DeltaGantry::update_next_unit_direction(float length)
{
  for (unsigned coord = 0; coord < 3; coord++) {
    float delta = last.cartesian[coord] - next.cartesian[coord];
    next.unit_direction[coord] = delta/length;
  }
}

float DeltaGantry::limit_speed(float requested_speed, float length) const
{
  float max_time_limit = 0;
  for (unsigned tower = 0; tower < towers.size(); tower++) {
    max_time_limit = std::max(max_time_limit,
      towers[tower].min_time_per_step*next.tower_steps[tower]);
  }
  for (unsigned extr = 0; extr < extruders.size(); extr++) {
    max_time_limit = std::max(max_time_limit,
      extruders[extr].min_time_per_step*next.extruder_steps[extr]);
  }
  float allowed_speed = length/max_time_limit;
  return std::min(requested_speed, allowed_speed);
}


float DeltaGantry::limit_acc(float requested_acc, float length) const
{
  float max_time2_limit = 0;
  for (unsigned tower = 0; tower < towers.size(); tower++) {
    max_time2_limit = std::max(max_time2_limit,
      towers[tower].min_time2_per_step*next.tower_steps[tower]);
  }
  for (unsigned extr = 0; extr < extruders.size(); extr++) {
    max_time2_limit = std::max(max_time2_limit,
      extruders[extr].min_time2_per_step*next.extruder_steps[extr]);
  }
  float allowed_acc = length/max_time2_limit;
  return std::min(requested_acc, allowed_acc);
}


float DeltaGantry::limit_jerk(float acc) const
{
  float cos_theta = 0;
  for (unsigned coord = 0; coord < 3; coord++) {
    cos_theta -= last.unit_direction[coord] * next.unit_direction[coord];
  }
  float sin_theta_d2 = sqrt(0.5f*(1.0f-cos_theta));
  return acc * junction_deviation * sin_theta_d2/(1.0f - sin_theta_d2);
}
