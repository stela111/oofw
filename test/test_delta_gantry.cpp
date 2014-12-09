#include "src/delta_gantry.h"
#include "src/planner.h"

#include <gtest/gtest.h>
#include <iostream>
#include <cmath>

static void print_move(const Gantry::LinearMove& move) {
  std::cout << "(";
  for (unsigned axis = 0; axis < move.steps.size(); ++axis) {
    if (axis > 0) {
      std::cout << ", ";
    }
    std::cout << move.steps.at(axis);
  }
  std::cout << ")"
	    << ", entry_speed = " << move.entry_speed
	    << ", cruise_speed = " << move.cruise_speed
	    << ", acceleration = " << move.acceleration
            << ", length = " << move.length
	    << std::endl;
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
  DeltaGantry::LinearMove move;
  move.steps.resize(axes.size() + towers.size());
  
  gantry.set_speed(30);
  gantry.set_cartesian(0,3.05);
  while(gantry.get_move(move)) {
    print_move(move);
  }

  gantry.set_cartesian(1,3);
  while(gantry.get_move(move)) {
    print_move(move);
  }
}
