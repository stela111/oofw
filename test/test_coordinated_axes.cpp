#include <gtest/gtest.h>

#include <src/coordinated_axes.h>
#include <fake/timer.h>
#include <fake/pin_io.h>

TEST(CoordTest, Init) {
  fake::Timer timer;
  CoordinatedAxes ca(&timer);
  fake::PinIo io;
  Stepper::Pins pins;
  pins.enable = io.make_pin("enable", true);
  pins.step = io.make_pin("step", true);
  pins.dir = io.make_pin("dir", true);
  pins.endstop = io.make_pin("endstop");

  Stepper::Config config;
  config.steps_per_mm = 10;
  config.min_us_per_step = 1000;
  config.min_us_sqr_per_step = 1000;
  Stepper stepper(&io, pins, config);

  auto ax1 = ca.add_axis(&stepper);
  
  ca.plan_target(ax1, 10);
  ca.queue_move(1e6f);
}
