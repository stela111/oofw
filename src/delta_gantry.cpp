#include "delta_gantry.h"
#include <cmath>
#include <cstdlib>
#include "planner.h"

namespace {
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
  , target_extruder_pos(axes.size()-towers.size())
  , more_moves(true)
  , steps(axes.size())
  , junction_deviation(.01)
  , requested_speed(0)
  , requested_acc(300)
  , max_length(.1)
{
  for (unsigned coord = 0; coord < 3; coord++) {
    target_cartesian[coord] = 0;
    next.cartesian[coord] = 0;
    next.unit_direction[coord] = 0;
  }

  next.extruder_pos = target_extruder_pos;
  next.steps.assign(axes.size(),0);
  update_next_steps();
  last = next;
}


void DeltaGantry::set_cartesian(unsigned index, float pos)
{
  if (index < 3) {
    target_cartesian[index] = pos;
  }
}


void DeltaGantry::set_extruder(unsigned index, float pos)
{
  if (index < target_extruder_pos.size()) {
    target_extruder_pos[index] = pos;
  }
}


void DeltaGantry::set_speed(float speed)
{
  requested_speed = speed;
}

  
bool DeltaGantry::push(Planner &planner)
{
  if (planner.is_buffer_full()) {
    return false;
  }
  
  float length = update_next_pos();
  update_next_steps();
  update_next_unit_direction(length);

  update_steps();
  float speed = limit_speed(requested_speed, length);
  float acc = limit_acc(requested_acc, length);
  float init_speed = limit_jerk(acc);

  planner.plan_move(steps, 
		    2*acc*length,
		    speed*speed,
		    init_speed*init_speed);
  last = next;

  return more_moves;
}


float DeltaGantry::update_next_pos()
{
  float length = get_move_length();

  more_moves = length > max_length;

  if (!more_moves) {
    // Last move. Copy target to next

    for (unsigned coord = 0; coord < 3; coord++) {
      next.cartesian[coord] = target_cartesian[coord];
    }
    next.extruder_pos = target_extruder_pos;

    return length;
  }

  float rel = max_length/length;
  for (unsigned coord = 0; coord < 3; coord++) {
    next.cartesian[coord] = last.cartesian[coord] + 
      (target_cartesian[coord] - last.cartesian[coord])*rel;
  }
  for (unsigned extr = 0; extr < target_extruder_pos.size(); ++extr) {
    next.extruder_pos[extr] = last.extruder_pos[extr] +
      (target_extruder_pos[extr] - last.extruder_pos[extr])*rel;
  }

  return max_length;
}


void DeltaGantry::update_next_steps()
{
  for (unsigned tower = 0; tower < towers.size(); ++tower) {
    float pos = calculate_tower_pos(next.cartesian, towers[tower]);
    next.steps[tower] = pos * axes[tower].steps_per_mm;
  }

  for (unsigned extr = 0; extr < next.extruder_pos.size(); ++extr) {
    next.steps[extr + towers.size()] = next.extruder_pos[extr] *
      axes[extr + towers.size()].steps_per_mm;
  }
}

void DeltaGantry::update_steps()
{
  for (unsigned axis = 0; axis < axes.size(); ++axis) {
    steps[axis] = next.steps[axis] - last.steps[axis];
  }
}


float DeltaGantry::get_move_length() const
{
  // Vector distance for cartesian coordinates
  float sqr_sum = 0;
  for (unsigned coord = 0; coord < 3; coord++) {
    float delta = target_cartesian[coord] - last.cartesian[coord];
    sqr_sum += delta*delta;
  }

  float length = sqrtf(sqr_sum);

  // Ensure length is not less than any axis distance
  // Skip tower coordinates since these are handled above
  for (unsigned extr = 0; extr < target_extruder_pos.size(); ++extr)
  {
    float delta = target_extruder_pos[extr] - last.extruder_pos[extr];
    length = std::max(length, std::abs(delta));
  }

  return length;
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
  float sin_theta_d2 = std::sqrt(0.5f*(1.0f-cos_theta));
  return acc * junction_deviation * sin_theta_d2/(1.0f - sin_theta_d2);
}
