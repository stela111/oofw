#include "src/scheduling_timer.h"

#include <gmock/gmock.h>

struct MockSchedulable : public Schedulable
{
  MOCK_METHOD0(on_timer, void());
};

struct MockBase {
  void set_callback(TimerCallback *) {}
  MOCK_METHOD0(frequency, uint32_t());
  MOCK_METHOD0(disable_interrupts, void());
  MOCK_METHOD0(enable_interrupts, void());
  MOCK_METHOD1(start_timer, void(uint16_t));
  MOCK_METHOD0(stop_timer, void());
  MOCK_METHOD0(current_timestamp, uint16_t());
};

class FakeTimer {
  FakeTimer(uint32_t freq)
    : freq(freq)
    , interrupts(true)
    , running(false)
    , timestamp(0)
    , next_timestamp(0)
    , callback(nullptr)
  {
  }

  void set_callback(TimerCallback *cb)
  {
    EXPECT_FALSE(interrupts);
    EXPECT_FALSE(running);
    callback = cb;
  }
  uint32_t frequency()
  {
    return freq;
  }

  uint16_t current_timestamp()
  {
    return timestamp;
  }

  void start_timer(uint16_t next)
  {
    EXPECT_FALSE(interrupts);

    next_timestamp = next;
    running = true;
  }
  void stop_timer()
  {
    EXPECT_FALSE(interrupts);

    running = false;
  }
  void disable_interrupts()
  {
    EXPECT_TRUE(interrupts);

    interrupts = false;
  }
  void enable_interrupts()
  {
    EXPECT_FALSE(interrupts);

    interrupts = true;
  }

  // Fake helpers
  void fire(uint16_t timestamp) {
    timestamp = timestamp;
    ASSERT_TRUE(callback != nullptr);
    callback->on_timer();
  }

  uint16_t get_next_timestamp()
  {
    return next_timestamp;
  }
private:
  uint32_t freq;
  bool interrupts;
  bool running;
  uint16_t timestamp;
  uint16_t next_timestamp;
  TimerCallback *callback;
};

using ::testing::Return;

TEST(SchedulableTest, single) {
  MockSchedulable task;
  SchedulingTimer<MockBase> s;

  ::testing::InSequence dummy;
  EXPECT_CALL(s.get_base(), disable_interrupts());
  EXPECT_CALL(s.get_base(), start_timer(10));
  EXPECT_CALL(s.get_base(), enable_interrupts());

  s.schedule(&task, 10);

  EXPECT_CALL(s.get_base(), current_timestamp()).WillOnce(Return(10));
  EXPECT_CALL(task, on_timer());
  EXPECT_CALL(s.get_base(), stop_timer());
  s.on_timer();

  EXPECT_CALL(s.get_base(), stop_timer());
  s.on_timer();
}

TEST(SchedulableTest, double) {
  MockSchedulable task1, task2;
  SchedulingTimer<MockBase> s;

  ::testing::InSequence dummy;
  EXPECT_CALL(s.get_base(), disable_interrupts());
  EXPECT_CALL(s.get_base(), start_timer(10));
  EXPECT_CALL(s.get_base(), enable_interrupts());
  s.schedule(&task1, 10);

  EXPECT_CALL(s.get_base(), disable_interrupts());
  EXPECT_CALL(s.get_base(), enable_interrupts());
  s.schedule(&task2, 20);

  EXPECT_CALL(s.get_base(), current_timestamp()).WillOnce(Return(10));
  EXPECT_CALL(task1, on_timer());
  EXPECT_CALL(s.get_base(), current_timestamp()).WillOnce(Return(10));
  EXPECT_CALL(s.get_base(), start_timer(20));
  s.on_timer();

  EXPECT_CALL(s.get_base(), current_timestamp()).WillOnce(Return(20));
  EXPECT_CALL(task2, on_timer());
  EXPECT_CALL(s.get_base(), stop_timer());
  s.on_timer();
}
