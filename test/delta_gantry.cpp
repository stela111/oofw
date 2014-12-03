#include "src/delta_gantry.h"
#include <gtest/gtest.h>

TEST(DeltaGantry, Init)
{
  std::vector<DeltaGantry::Axis> axes;
  std::vector<DeltaGantry::Tower> towers;
  axes.push_back(DeltaGantry::Axis{100, 1e-3, 1e-4});
  axes.push_back(DeltaGantry::Axis{100, 1e-3, 1e-4});
  axes.push_back(DeltaGantry::Axis{100, 1e-3, 1e-4});
  axes.push_back(DeltaGantry::Axis{100, 1e-3, 1e-4});
  towers.push_back(DeltaGantry::Tower{{5,0,0},5});

  DeltaGantry gantry(axes, towers);
  
  gantry.set_speed(100);
  gantry.set_cartesian(0,1);
  gantry.move();
}
