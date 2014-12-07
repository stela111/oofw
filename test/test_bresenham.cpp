#include <src/bresenham.h>
#include <gtest/gtest.h>

TEST(Bresenham, 5_10_slope) {
  Bresenham b(5,10);
  
  for (int i=0; i < 10; i++) {
    EXPECT_TRUE(b.tick());
    EXPECT_FALSE(b.tick());
  }
}

TEST(Bresenham, 2_5_slope) {
  Bresenham b(2,5);
    
  EXPECT_TRUE(b.tick());
  EXPECT_FALSE(b.tick());
  EXPECT_TRUE(b.tick());
  EXPECT_FALSE(b.tick());
  EXPECT_FALSE(b.tick());
}

TEST(Bresenham, zero_slope) {
  Bresenham b(0,5);
  
  EXPECT_FALSE(b.tick());
  EXPECT_FALSE(b.tick());
  EXPECT_FALSE(b.tick());
  EXPECT_FALSE(b.tick());
  EXPECT_FALSE(b.tick());
}
