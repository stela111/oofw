/**
      This file is part of Oofw (http://github.org/stela111/oofw/). The planner part is heavily based on Grbl (https://github.com/simen/grbl).
      Oofw is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Oofw is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Oofw. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TRAPEZOIDGENERATOR_H
#define TRAPEZOIDGENERATOR_H

#include <cstdint>

// Helper class that handles the ramp algorithm
class Ramp {
public:
  Ramp(std::uint32_t c0, std::int32_t n);
  
  std::uint32_t getDelay();
  std::uint32_t nextDelay();
  void reverseAcc();
private:
  std::int32_t c;
  std::int32_t n;
  std::int32_t remainder;
};

// Precalculates all values for the TrapezoidGenerator
struct TrapezoidParameters {
  TrapezoidParameters();
  TrapezoidParameters(std::uint32_t steps,
		      float entryRate,
		      float exitRate,
		      float nominalRate,
		      float timerFreq,
		      float acc);

  std::uint32_t steps;
  std::int32_t c0;
  std::int32_t n0;
  std::uint32_t accelerateUntil;
  std::uint32_t decelerateAfter;
};

/// TrapezoidGenerator produces delays for trapezoid shaped pulse frequency.
/**
   Implementation based on "Generate stepper-motor speed profiles in real time"
   by D. Austin, article in Embedded Systems Programming January 2005.
   See http://fab.cba.mit.edu/classes/MIT/961.09/projects/i0/Stepper_Motor_Speed_Profile.pdf
 */
class TrapezoidGenerator {
public:
    TrapezoidGenerator(TrapezoidParameters params = TrapezoidParameters());

    /// Calculate next delay.
    /** Returns 0 if trapezoid is completed.
     */
    std::uint32_t next_delay();

    /// Returns true when last delay has been calculated by next_delay().
    bool is_done() {
      return steps == step;
    }
private:
    std::uint32_t accelerateUntil;
    std::uint32_t decelerateAfter;
    std::uint32_t steps;
    std::uint32_t step;
    Ramp ramp;
};

#endif
