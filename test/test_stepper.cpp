#include <gtest/gtest.h>

#include <src/stepper.h>
#include <fake/pin_io.h>

class StepperTest : public ::testing::Test
{
public:
  virtual void SetUp() {
    pins.enable = io.make_pin("enable", true);
    pins.step = io.make_pin("step", true);
    pins.dir = io.make_pin("dir", true);
    pins.endstop = io.make_pin("endstop");

    // Ignore config. Not used here
  }

protected:
  fake::PinIo io;
  Stepper::Pins pins;
  Stepper::Config config;
};


TEST_F(StepperTest, Init) {
  Stepper stepper(&io, pins, config);
  EXPECT_FALSE(io.get(pins.enable));
  EXPECT_FALSE(io.get(pins.step));
  EXPECT_FALSE(io.get(pins.dir));
  EXPECT_EQ(Stepper::State::DISABLED, stepper.state());
}


TEST_F(StepperTest, EnableAndDisable) {
  Stepper stepper(&io, pins, config);

  stepper.enable();
  EXPECT_TRUE(io.get(pins.enable));
  EXPECT_EQ(Stepper::State::ACTIVE, stepper.state());

  stepper.disable();
  EXPECT_FALSE(io.get(pins.enable));
  EXPECT_EQ(Stepper::State::DISABLED, stepper.state());
}

TEST_F(StepperTest, Step) {
  Stepper stepper(&io, pins, config);

  stepper.step();
  EXPECT_FALSE(io.get(pins.step));
  EXPECT_EQ(Stepper::State::DISABLED, stepper.state());
  stepper.enable();
  stepper.step();
  EXPECT_EQ(Stepper::State::STEPPING, stepper.state());
  EXPECT_TRUE(io.get(pins.step));
  stepper.unstep();
  EXPECT_FALSE(io.get(pins.step));
}

TEST_F(StepperTest, Dir) {
  Stepper stepper(&io, pins, config);

  stepper.set_direction(false);
  EXPECT_FALSE(io.get(pins.dir));
  stepper.set_direction(true);
  EXPECT_TRUE(io.get(pins.dir));
}

TEST_F(StepperTest, Position) {
  Stepper stepper(&io, pins, config);

  stepper.enable();
  stepper.set_position(100);
  EXPECT_EQ(100, stepper.position());

  stepper.set_direction(true);
  stepper.step();
  stepper.step();
  stepper.unstep();
  EXPECT_EQ(101, stepper.position());  
  stepper.set_direction(false);
  stepper.step();
  stepper.unstep();
  EXPECT_EQ(100, stepper.position());
}


TEST_F(StepperTest, StopOnEndstop) {
  Stepper stepper(&io, pins, config);

  stepper.set_position(0);
  stepper.set_direction(true);
  stepper.enable();
  stepper.stop_on_endstop(true);
  io.set(pins.endstop);
  stepper.step();
  EXPECT_TRUE(io.get(pins.step));
  stepper.unstep();
  EXPECT_EQ(Stepper::State::STOPPED, stepper.state());
  EXPECT_EQ(1, stepper.position());
  stepper.step();
  EXPECT_FALSE(io.get(pins.step));
  EXPECT_EQ(1, stepper.position());
  stepper.unstep();
}
