#include "src/delta_gantry.h"
#include "src/planner.h"

#include <gtest/gtest.h>
#include <iostream>
#include <cmath>

static void print_move(Planner& planner) {
  auto steps = planner.get_current_steps();
  std::cout << "(";
  for (unsigned axis = 0; axis < steps->size(); ++axis) {
    if (axis > 0) {
      std::cout << ", ";
    }
    std::cout << steps->at(axis);
  }
  std::cout << ")"
	    << ", entry = " << std::sqrt(planner.get_current_entry_speed_sqr())
	    << ", nominal = " << std::sqrt(planner.get_current_speed_sqr())
	    << ", exit = " << std::sqrt(planner.get_current_exit_speed_sqr())
	    << std::endl;
  planner.next_move();
}

static void print_moves(Planner& planner) {
  while(planner.get_current_steps()) {
    print_move(planner);
  }
}

TEST(DeltaGantry, Init)
{
  std::vector<DeltaGantry::Axis> axes;
  std::vector<DeltaGantry::Tower> towers;
  axes.push_back(DeltaGantry::Axis{100, 1e-4, 1e-6});
  axes.push_back(DeltaGantry::Axis{100, 1e-4, 1e-6});
  axes.push_back(DeltaGantry::Axis{100, 1e-4, 1e-6});
  axes.push_back(DeltaGantry::Axis{100, 1e-4, 1e-6});
  towers.push_back(DeltaGantry::Tower{{5,0,0},10});
  towers.push_back(DeltaGantry::Tower{{0,5,0},10});
  towers.push_back(DeltaGantry::Tower{{-5,0,0},10});

  DeltaGantry gantry(axes, towers);
  Planner planner(16, axes.size());
  
  gantry.set_speed(30);
  gantry.set_cartesian(0,3);
  do {
    if (planner.is_buffer_full()) {
      print_move(planner);
    }
  } while(gantry.push(planner));

  gantry.set_cartesian(1,3);
  do {
    if (planner.is_buffer_full()) {
      print_move(planner);
    }
  } while(gantry.push(planner));
  print_moves(planner);
}
