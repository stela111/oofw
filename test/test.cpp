#include <gtest/gtest.h>
#include <src/planner.h>

TEST(Planner, PlanOne) {
  Planner planner(2, 1);
  std::vector<int> steps(1);

  EXPECT_FALSE(planner.check_full_buffer());
  EXPECT_EQ(0, planner.get_current_move());
  
  planner.plan_move(steps, 9,9,9);
  EXPECT_TRUE(planner.check_full_buffer());

  auto *move = planner.get_current_move();
  EXPECT_TRUE(move != nullptr);
  EXPECT_EQ(0, planner.get_current_entry_speed_sqr());
  EXPECT_EQ(0, planner.get_current_exit_speed_sqr());
  planner.next_move();

  EXPECT_FALSE(planner.check_full_buffer());
  EXPECT_EQ(nullptr, planner.get_current_move());
}

TEST(Planner, PlanAccLimit) {
  Planner planner(16,1);
  std::vector<int> steps(1);

  planner.plan_move(steps, 4, 9, 0);
  planner.plan_move(steps, 6, 9, 9);
  planner.plan_move(steps, 5, 9, 9);
  planner.plan_move(steps, 5, 9, 9);
  EXPECT_EQ(0, planner.get_current_entry_speed_sqr());
  EXPECT_EQ(4, planner.get_current_exit_speed_sqr());
  planner.next_move();
  EXPECT_EQ(4, planner.get_current_entry_speed_sqr());  
  planner.next_move();
  EXPECT_EQ(9, planner.get_current_entry_speed_sqr());
  planner.next_move();
  EXPECT_EQ(5, planner.get_current_entry_speed_sqr());
}


TEST(Planner, PlanNominalLimited) {
  Planner planner(16,1);
  std::vector<int> steps(1);

  planner.plan_move(steps, 9, 9, 0);
  planner.plan_move(steps, 9, 3, 9);
  planner.plan_move(steps, 9, 7, 9);
  planner.plan_move(steps, 9, 9, 9);
  EXPECT_EQ(0, planner.get_current_entry_speed_sqr());
  planner.next_move();
  EXPECT_EQ(3, planner.get_current_entry_speed_sqr());
  planner.next_move();
  EXPECT_EQ(3, planner.get_current_entry_speed_sqr());
  planner.next_move();
  EXPECT_EQ(7, planner.get_current_entry_speed_sqr());
}


TEST(Planner, PlanInitialLimited) {
  Planner planner(16, 1);
  std::vector<int> steps(1);

  planner.plan_move(steps, 5, 9, 9);
  planner.plan_move(steps, 5, 9, 3);
  planner.plan_move(steps, 5, 9, 4);
  EXPECT_EQ(0, planner.get_current_entry_speed_sqr());
  planner.next_move();
  EXPECT_EQ(3, planner.get_current_entry_speed_sqr());
  planner.next_move();
  EXPECT_EQ(4, planner.get_current_entry_speed_sqr());
}
