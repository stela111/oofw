#include <gtest/gtest.h>

#include <src/trapezoid_ticker.h>
#include <src/stepper.h>
#include <fake/timer.h>
#include <fake/pin_io.h>

class TrapezoidTest : public ::testing::Test
{
public:
  virtual void SetUp() {
    pins.enable = io.make_pin("enable", true);
    pins.step = io.make_pin("step", true);
    pins.dir = io.make_pin("dir", true);
    pins.endstop = io.make_pin("endstop");
    steppers.push_back(Stepper(&io, pins));

    stepperPtrs.push_back(&steppers[0]);
  }

protected:
  fake::Timer timer;
  fake::PinIo io;
  Stepper::Pins pins;
  std::vector<Stepper> steppers;
  std::vector<Stepper*> stepperPtrs;
};

TEST_F(TrapezoidTest, simple) {
  TrapezoidTicker ticker(stepperPtrs, &timer);
  steppers[0].enable();
  steppers[0].set_direction(true);
  ticker.generate(std::vector<int>({6}),
		  10,
		  0,
		  0,
		  10,
		  15);

  std::uint32_t delay;
  while((delay = timer.fake_next())) {
    std::cout << steppers[0].position() << ", " << delay*1e-6 << std::endl;
  }
  EXPECT_EQ(6, steppers[0].position());
}
