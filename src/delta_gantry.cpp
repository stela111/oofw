#include "delta_gantry.h"
#include <cmath>
#include <cstdlib>

namespace {
  const unsigned num_towers = 3;
  
  float calculate_tower_pos(float cartesian[3],
			    const DeltaGantry::Tower& tower) {
    float dist[3];
    for (unsigned coord = 0; coord < 3; coord++) {
      dist[coord] = cartesian[coord] - tower.origin[coord];
    }
    float xy_dist2 = dist[0]*dist[0] + dist[1]*dist[1];
    float arm_length2 = tower.arm_length*tower.arm_length;
    bool out_of_range = xy_dist2 > arm_length2;
    return dist[2] + out_of_range ? 0 : sqrtf(arm_length2 - xy_dist2);
  }
};

DeltaGantry::DeltaGantry(const std::vector<Axis>& axes,
			 const std::vector<Tower>& towers)
  : towers(towers)
  , axes(axes)
  , steps(axes.size())
  , requested_speed(0)
  , requested_acc(1000)
  , planner(16, axes.size())
{
  next.steps.assign(axes.size(),0);
  next.cartesian[0] = 0;
  next.cartesian[1] = 0;
  next.cartesian[2] = 0;  
  last = next;
}


void DeltaGantry::set_cartesian(unsigned index, float pos)
{
  if (index < 3) {
    next.cartesian[index] = pos;
  }
}


void DeltaGantry::set_extruder(unsigned index, float pos)
{
  unsigned axis = index + towers.size();
  if (axis < axes.size()) {
    next.steps[axis] = pos*axes[axis].steps_per_mm;
  }
}


void DeltaGantry::set_speed(float speed)
{
  requested_speed = speed;
}

  
void DeltaGantry::move()
{
  update_steps();
  float length = get_move_length();
  update_next_unit_direction(length);
  float speed = limit_speed(requested_speed, length);
  float acc = limit_acc(requested_acc, length);
  float init_speed = limit_jerk(acc);
  planner.plan_move(steps, 
		    2*acc*length,
		    speed*speed,
		    init_speed*init_speed);
  last = next;
}


void DeltaGantry::update_steps()
{
  for (unsigned tower = 0; tower < towers.size(); ++tower) {
    float pos = calculate_tower_pos(next.cartesian, towers[tower]);
    next.steps[tower] = pos * axes[tower].steps_per_mm;
  }
  for (unsigned axis = 0; axis < axes.size(); ++axis) {
    steps[axis] = next.steps[axis] - last.steps[axis];
  }
}


float DeltaGantry::get_move_length() const
{
  // Vector distance for cartesian coordinates
  float sqr_sum = 0;
  for (unsigned coord = 0; coord < 3; coord++) {
    float delta = next.cartesian[coord] - last.cartesian[coord];
    sqr_sum += delta*delta;
  }

  // Ensure length is not less than any axis distance
  // Skip tower coordinates since these are handled above
  for (unsigned axis = towers.size(); axis < axes.size(); ++axis)
  {
    float delta = std::abs(steps[axis])/axes[axis].steps_per_mm;
    sqr_sum = std::max(sqr_sum, delta*delta);
  }

  return sqrtf(sqr_sum);
}


void DeltaGantry::update_next_unit_direction(float length)
{
  for (unsigned coord = 0; coord < 3; coord++) {
    float delta = next.cartesian[coord] - last.cartesian[coord];
    next.unit_direction[coord] = delta/length;
  }
}

float DeltaGantry::limit_speed(float requested_speed, float length) const
{
  float max_time_limit = 0;
  for (unsigned axis = 0; axis < axes.size(); ++axis) {
    float axes_min_time = axes[axis].min_time_per_step*std::abs(steps[axis]);
    max_time_limit = std::max(max_time_limit, axes_min_time);
  }
  float allowed_speed = length/max_time_limit;
  return std::min(requested_speed, allowed_speed);
}


float DeltaGantry::limit_acc(float requested_acc, float length) const
{
  float max_time2_limit = 0;
  for (unsigned axis = 0; axis < axes.size(); axis++) {
    float axis_min_time2 = axes[axis].min_time2_per_step*std::abs(steps[axis]);
    max_time2_limit = std::max(max_time2_limit, axis_min_time2);
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
