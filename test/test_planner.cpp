#include <gtest/gtest.h>
#include <src/planner.h>

TEST(Planner, PlanOne) {
  Planner planner(2, 1);
  std::vector<int> steps(1);

  EXPECT_FALSE(planner.is_buffer_full());
  EXPECT_EQ(nullptr, planner.get_current_move());
  
  planner.plan_move(steps, 1, 1, 1, 0);
  EXPECT_TRUE(planner.is_buffer_full());

  auto *move = planner.get_current_move();
  EXPECT_TRUE(move != nullptr);
  EXPECT_EQ(0, planner.get_current_entry_speed_sqr());
  EXPECT_EQ(0, planner.get_current_exit_speed_sqr());
  planner.next_move();

  EXPECT_FALSE(planner.is_buffer_full());
  EXPECT_EQ(nullptr, planner.get_current_move());
}

TEST(Planner, PlanAccLimit) {
  Planner planner(16,1);
  std::vector<int> steps(1);

  planner.plan_move(steps, 1, 10, 2, 10);
  planner.plan_move(steps, 1, 10, 1, 10);
  planner.plan_move(steps, 1, 10, 2, 10);
  planner.plan_move(steps, 2, 10, 2, 10);
  EXPECT_EQ(0, planner.get_current_entry_speed_sqr());
  EXPECT_EQ(4, planner.get_current_exit_speed_sqr());
  planner.next_move();
  EXPECT_EQ(4, planner.get_current_entry_speed_sqr());  
  planner.next_move();
  EXPECT_EQ(6, planner.get_current_entry_speed_sqr());
  planner.next_move();
  EXPECT_EQ(8, planner.get_current_entry_speed_sqr());
}


TEST(Planner, PlanNominalLimited) {
  Planner planner(16,1);
  std::vector<int> steps(1);

  planner.plan_move(steps, 1, 5, 10, 100);
  planner.plan_move(steps, 1, 3, 10, 100);
  planner.plan_move(steps, 1, 2, 10, 100);
  planner.plan_move(steps, 1, 4, 10, 100);
  EXPECT_EQ(0, planner.get_current_entry_speed_sqr());
  planner.next_move();
  EXPECT_EQ(9, planner.get_current_entry_speed_sqr());
  planner.next_move();
  EXPECT_EQ(4, planner.get_current_entry_speed_sqr());
  planner.next_move();
  EXPECT_EQ(4, planner.get_current_entry_speed_sqr());
}


TEST(Planner, PlanInitialLimited) {
  Planner planner(16, 1);
  std::vector<int> steps(1);

  planner.plan_move(steps, 1, 10, 10, 0);
  planner.plan_move(steps, 1, 10, 10, 2);
  planner.plan_move(steps, 1, 10, 10, 1);
  EXPECT_EQ(0, planner.get_current_entry_speed_sqr());
  planner.next_move();
  EXPECT_EQ(4, planner.get_current_entry_speed_sqr());
  planner.next_move();
  EXPECT_EQ(1, planner.get_current_entry_speed_sqr());
}

