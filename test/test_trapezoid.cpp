#include <gtest/gtest.h>

#include <src/trapezoid_ticker.h>
#include <src/stepper.h>
#include <fake/timer.h>
#include <fake/pin_io.h>

class TrapezoidTest : public ::testing::Test
{
public:
  virtual void SetUp() {
    for (unsigned stepper = 0; stepper < 4; stepper++) {
      Stepper::Pins pins;
      std::string name = std::to_string(stepper);
      pins.enable = io.make_pin("enable" + name, true);
      pins.step = io.make_pin("step" + name, true);
      pins.dir = io.make_pin("dir" + name, true);
      pins.endstop = io.make_pin("endstop" + name);
      steppers.emplace_back(Stepper(&io, pins));
    }

    for (auto& stepper : steppers) {
      stepperPtrs.push_back(&stepper);
      stepper.enable();
    }
  }

protected:
  fake::Timer timer;
  fake::PinIo io;
  std::vector<Stepper> steppers;
  std::vector<Stepper*> stepperPtrs;
};

TEST_F(TrapezoidTest, simple) {
  TrapezoidTicker ticker(stepperPtrs, &timer);

  std::vector<int> steps{1,2,-3,10};
  float length = 1;
  float cruise_speed = 10;
  float entry_speed = 1;
  float exit_speed = 5;
  float acceleration = 100;

  float rate = 3e2;
  unsigned events = static_cast<int>(length*rate/cruise_speed);
  float factor = events/length;

  for (unsigned ind = 0; ind < steps.size(); ind++) {
    steppers[ind].set_direction(steps[ind]>0);
    //    steps[ind] = std::abs(steps[ind]);
  }
  
  ticker.generate(steps,
		  events,
		  entry_speed*factor,
		  exit_speed*factor,
		  cruise_speed*factor,
		  acceleration*factor);

  std::uint32_t delay = 0;
  std::uint32_t time = 0;
  std::cout << time << ": ";
  while((delay = timer.fake_next())) {
    for (unsigned ind = 0; ind < steps.size(); ind++) {
      if (ind > 0) std::cout << ", ";
      std::cout << steppers[ind].position();
    }
    time += delay;
    std::cout << std::endl << time << ": ";
  }
  std::cout << std::endl;

  for (unsigned ind = 0; ind < steps.size(); ind++) {
    EXPECT_EQ(steps[ind], steppers[ind].position());
  }
}
