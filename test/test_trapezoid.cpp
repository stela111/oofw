#include <gtest/gtest.h>

#include <src/trapezoid_ticker.h>
#include <src/planner.h>
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
  Planner planner(16,steppers.size());

  std::vector<int> steps{1,2,-3,10};
  std::vector<int> steps2{-2,-3,2,-9};

  planner.plan_move(steps, 1, 10, 100, 1);
  planner.plan_move(steps2, 1, 20, 100, 1);
  ticker.start(&planner);

  std::uint32_t delay = 0;
  std::uint32_t time = 0;
  std::cout << time << ": ";
  while((delay = timer.fake_next())) {
    for (unsigned ind = 0; ind < steps.size(); ind++) {
      if (ind > 0) std::cout << ", ";
      std::cout << steppers[ind].position();
    }
    time += delay;
    std::cout << std::endl << delay << ": ";
  }
  std::cout << std::endl;

  for (unsigned ind = 0; ind < steps.size(); ind++) {
    EXPECT_EQ(steps[ind]+steps2[ind], steppers[ind].position());
  }
}
