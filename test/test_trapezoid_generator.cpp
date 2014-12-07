#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include <src/trapezoid_generator.h>

namespace {
  const uint32_t reference[] = {1000, 600, 467, 395, 349, 316, 291, 271, 254, 241, 229};

  template<typename T, std::size_t N>
  std::vector<T> makeVec(const T (&vec)[N]) {
    return std::vector<T>(vec, vec+N);
  }
    
  void testTrapezoid(TrapezoidParameters param, const std::vector<uint32_t> &reference) {
    TrapezoidGenerator g(param);
    ASSERT_EQ(param.steps, reference.size());
    
    size_t count = 0;
    do {
      ASSERT_GT(param.steps, count);
      EXPECT_EQ(reference[count], g.next_delay()) << count;
      count++;
    }
    while (!g.is_done());

    EXPECT_EQ(param.steps, count);
  }
}

TEST(Ramp, verify_with_reference_data) {
  Ramp ramp(1000, 0);
  
  int ind = 0;
  
  EXPECT_EQ(reference[ind], ramp.getDelay());
  while (unsigned(++ind) < sizeof(reference)/sizeof(*reference)) {
    EXPECT_EQ(reference[ind], ramp.nextDelay());  
  }
  
  ramp.reverseAcc();
  EXPECT_EQ(reference[--ind], ramp.getDelay());

  while (--ind >= 0) {
    EXPECT_EQ(reference[ind], ramp.nextDelay());  
  }
}

TEST(TrapezoidGenerator, constant_speed) {
  const uint32_t steps = 10;
  const float v = 100;
  const float f = 1e4f;
  TrapezoidParameters p(steps, v, v, v, f, 1000);
  
  std::vector<uint32_t> ref(steps, f/v);

  testTrapezoid(p, ref);  
}

TEST(TrapezoidGenerator, start_stop_no_cruising) {
  const uint32_t steps = 10;
  const float v = 100;
  const float f = 1e4f;
  const float acc = 2.0/powf(1000/f/0.676,2);
  TrapezoidParameters p(steps, 0, 0, v, f, acc);

  EXPECT_LT(steps/2, v*v/(2*acc)) << "Input params cause cruising";

  uint32_t ref[] = {
    reference[0],
    reference[1],
    reference[2],
    reference[3],
    reference[4],
    reference[5],
    reference[4],
    reference[3],
    reference[2],
    reference[1]
  };
    
  testTrapezoid(p, makeVec(ref));  
}

TEST(TrapezoidGenerator, start_stop_with_cruising) {
  const uint32_t steps = 10;
  const float v = 25.3; // Velocity corresponding to ref[3]
  const float f = 1e4f;
  const float acc = 2.0/powf(1000/f/0.676,2);
  TrapezoidParameters p(steps, 0, 0, v, f, acc);

  uint32_t ref[] = {
    reference[0],
    reference[1],
    reference[2],
    reference[3],
    reference[3],
    reference[3],
    reference[3],
    reference[3],
    reference[2],
    reference[1]
  };
    
  testTrapezoid(p, makeVec(ref));  
}

TEST(TrapezoidGenerator, running_to_stop_with_cruising) {
  const uint32_t steps = 10;
  const float v = 25.3;
  const float f = 1e4f;
  const float acc = 2.0/powf(1000/f/0.676,2);
  TrapezoidParameters p(steps, v, 0, v, f, acc);

  const uint32_t ref[] = {
    reference[3],
    reference[3],
    reference[3],
    reference[3],
    reference[3],
    reference[3],
    reference[3],
    reference[3],
    reference[2],
    reference[1]+1
  };

  testTrapezoid(p, makeVec(ref));  
}

TEST(TrapezoidGenerator, decel_only) {
  const uint32_t steps = 3;
  const float v = 25.3;
  const float f = 1e4f;
  const float acc = 2.0/powf(1000/f/0.676,2);
  TrapezoidParameters p(steps, v, 0, v, f, acc);

  std::vector<uint32_t> ref;
  ref.push_back(reference[3]);
  ref.push_back(reference[2]);
  ref.push_back(reference[1]+1); // Rounding different due to remainder calculations

  testTrapezoid(p, ref);
}

TEST(TrapezoidGenerator, acc_only) {
  const uint32_t steps = 3;
  const float v = 100;
  const float f = 1e4f;
  const float acc = 2.0/powf(1000/f/0.676,2);
  TrapezoidParameters p(steps, 0, v, v, f, acc);

  std::vector<uint32_t> ref;
  ref.push_back(reference[0]);
  ref.push_back(reference[1]);
  ref.push_back(reference[2]);

  testTrapezoid(p, ref);
}

TEST(TrapezoidGenerator, start_to_running_with_cruising) {
  const uint32_t steps = 10;
  const float v = 25.3;
  const float f = 1e4f;
  const float acc = 2.0/powf(1000/f/0.676,2);
  TrapezoidParameters p(steps, 0, v, v, f, acc);

  const uint32_t ref[] = {
    reference[0],
    reference[1],
    reference[2],
    reference[3],
    reference[3],
    reference[3],
    reference[3],
    reference[3],
    reference[3],
    reference[3],
  };

  testTrapezoid(p, makeVec(ref));  
}
