#include "coordinated_axes.h"

#include <cstdlib>

CoordinatedAxes::CoordinatedAxes(Timer *timer)
  : timer_(timer)
  , state_(State::IDLE)
  , planner_(16)
{
}

unsigned CoordinatedAxes::add_axis(Stepper *stepper)
{
  Axis axis;

  axis.stepper = stepper;
  axis.position = 0;
  axis.next_steps = 0;

  unsigned axis_id = axes_.size();
  axes_.push_back(axis);
  return axis_id;
}

void CoordinatedAxes::plan_target(unsigned axis, float position_mm)
{
  float steps_per_mm = axes_[axis].stepper->config().steps_per_mm;
  std::int32_t position = position_mm*steps_per_mm;
  axes_[axis].next_steps = position - axes_[axis].position;
}

void CoordinatedAxes::queue_move(std::uint32_t us)
{
  std::uint32_t limited_us = us;
  float limited_us2 = 1e10f;

  // Check limits for each axis
  for (const auto& axis: axes_) {
    std::int32_t steps = std::abs(axis.next_steps);

    // Limit time based on planned steps and min interval between steps
    std::uint32_t min_us_per_step = axis.stepper->config().min_us_per_step;
    limited_us = std::min(min_us_per_step*steps, limited_us);

    // Limit 
    float min_us2_per_step = axis.stepper->config().min_us_sqr_per_step;
    limited_us2 = std::min(min_us2_per_step*steps, limited_us2);
  }

  // @todo: Add jerk

  float max_speed2 = 1e6f*1e6f/(limited_us*limited_us);

  bool start = planner_.get_current_move() == nullptr;

  planner_.plan_move(Steps(), 1e6f/limited_us2, max_speed2, 0);

  if (start) {
    timer_->reset_time();
    //    next_segment();
  }
}


void CoordinatedAxes::on_timer() {
}
