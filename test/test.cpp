#include <gtest/gtest.h>
#include <src/planner.h>

TEST(Planner, PlanOne) {
  Planner planner(2);

  EXPECT_FALSE(planner.check_full_buffer());
  EXPECT_EQ(0, planner.get_current_move());
  
  planner.plan_move(Steps(), 9,9,9);
  EXPECT_TRUE(planner.check_full_buffer());

  const Planner::Move *move = planner.get_current_move();
  EXPECT_EQ(0, move->entry_speed_sqr);
  EXPECT_EQ(0, planner.get_current_exit_speed_sqr());
  planner.next_move();

  EXPECT_FALSE(planner.check_full_buffer());
  EXPECT_EQ(0, planner.get_current_move());
}

TEST(Planner, PlanAccLimit) {
  Planner planner(16);

  planner.plan_move(Steps(), 4, 9, 0);
  planner.plan_move(Steps(), 6, 9, 9);
  planner.plan_move(Steps(), 5, 9, 9);
  planner.plan_move(Steps(), 5, 9, 9);
  const Planner::Move *move = planner.get_current_move();
  EXPECT_EQ(0, move->entry_speed_sqr);
  EXPECT_EQ(4, planner.get_current_exit_speed_sqr());
  planner.next_move();
  move = planner.get_current_move();
  EXPECT_EQ(4, move->entry_speed_sqr);  
  planner.next_move();
  move = planner.get_current_move();
  EXPECT_EQ(9, move->entry_speed_sqr);  
  planner.next_move();
  move = planner.get_current_move();
  EXPECT_EQ(5, move->entry_speed_sqr);  
}


TEST(Planner, PlanNominalLimited) {
  Planner planner(16);

  planner.plan_move(Steps(), 9, 9, 0);
  planner.plan_move(Steps(), 9, 3, 9);
  planner.plan_move(Steps(), 9, 7, 9);
  planner.plan_move(Steps(), 9, 9, 9);
  const Planner::Move *move = planner.get_current_move();
  EXPECT_EQ(0, move->entry_speed_sqr);
  planner.next_move();
  move = planner.get_current_move();
  EXPECT_EQ(3, move->entry_speed_sqr);  
  planner.next_move();
  move = planner.get_current_move();
  EXPECT_EQ(3, move->entry_speed_sqr);  
  planner.next_move();
  move = planner.get_current_move();
  EXPECT_EQ(7, move->entry_speed_sqr);  
}


TEST(Planner, PlanInitialLimited) {
  Planner planner(16);

  planner.plan_move(Steps(), 5, 9, 9);
  planner.plan_move(Steps(), 5, 9, 3);
  planner.plan_move(Steps(), 5, 9, 4);
  const Planner::Move *move = planner.get_current_move();
  EXPECT_EQ(0, move->entry_speed_sqr);
  planner.next_move();
  move = planner.get_current_move();
  EXPECT_EQ(3, move->entry_speed_sqr);  
  planner.next_move();
  move = planner.get_current_move();
  EXPECT_EQ(4, move->entry_speed_sqr);  
}
