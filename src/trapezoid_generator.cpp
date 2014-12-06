#include "trapezoid_generator.h"
#include <cmath>

Ramp::Ramp(std::uint32_t c0, std::int32_t n)
 : c(c0)
 , n(n)
 , remainder(0)
{
}
  
std::uint32_t Ramp::getDelay() {
  return c;
}

std::uint32_t Ramp::nextDelay() {
  n += 1;
  std::int32_t nom = 2*c + remainder;
  std::int32_t den = 4*n + 1;
  remainder = nom % den;
  c -= nom / den;

  // Correction to ensure acc and dec ramps are identical
  if (den < 0 && remainder > 0) {
    c += 1;
    remainder += den;
  }

  return c;
}

void Ramp::reverseAcc() {
  // General algorithm: (n1+0.5)*a1 = (n2+0.5)*a2
  // In this case, a2 = -a1, and the equation reduces to:
  n = -n - 1;
  remainder = -remainder;
}


static float acc_steps(float v, float acc) {
  return (v*v)/(2.0f*acc);
}

static float initial_c(float f, float acc) {
  return 0.676f*f*sqrtf(2.0f/acc);
}

static std::uint32_t max(std::int32_t a, std::int32_t b) {
  return (a>b)?a:b;
}

TrapezoidParameters::TrapezoidParameters()
 : steps(0)
 , c0(0)
 , n0(0)
 , accelerateUntil(0)
 , decelerateAfter(0)
 {}

TrapezoidParameters::TrapezoidParameters(std::uint32_t steps,
					 float entryRate,
					 float exitRate,
					 float nominalRate,
					 float timerFreq,
					 float acc)
{
  // Steps to accelerate from zero to respective speeds
  // Not sure about ceil/floor here
  std::uint32_t stepsToNominal = floor(acc_steps(nominalRate, acc));
  std::uint32_t stepsToEntry = floor(acc_steps(entryRate, acc));
  std::uint32_t stepsToExit = floor(acc_steps(exitRate, acc));
  
  // These are the actual steps we need to [ac,de]celerate
  std::uint32_t accSteps = stepsToNominal - stepsToEntry;
  std::uint32_t decSteps = stepsToNominal - stepsToExit;
  
  std::int32_t plateauSteps = steps - accSteps - decSteps;
  if (plateauSteps < 0) {
    // No cruising, we need to reduce acc and decSteps appropriately.
    std::int32_t fullRampSteps = stepsToEntry + steps + stepsToExit;
    accSteps = max(fullRampSteps/2 - stepsToEntry, 0);
    plateauSteps = 0;
  }
  
  this->accelerateUntil = accSteps;
  this->decelerateAfter = accSteps + plateauSteps;
  this->steps = steps;
  
  // Setup ramp params
  this->n0 = stepsToEntry;
  if (stepsToEntry == 0) {
    this->c0 = round(initial_c(timerFreq, acc));
  }
  else {
    this->c0 = round(timerFreq/entryRate);
  }
}

TrapezoidGenerator::TrapezoidGenerator(TrapezoidParameters params)
 : accelerateUntil(params.accelerateUntil)
 , decelerateAfter(params.decelerateAfter)
 , steps(params.steps)
 , step(0)
 , ramp(params.c0, params.n0)
{
}

std::uint32_t TrapezoidGenerator::next_delay() {
  if (step == steps) {
    return 0;
  }

  step++;
  std::uint32_t current = ramp.getDelay();
  
  if (step > accelerateUntil && step <= decelerateAfter) {
    // Cruising
    return current;
  }
  else if (step == decelerateAfter+1) {
    // First deceleration step
    ramp.reverseAcc();
  }
  ramp.nextDelay();
  return current;
}
