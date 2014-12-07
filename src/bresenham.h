#ifndef BRESENHAM_H
#define BRESENHAM_H

/// Bresenham's line drawing algorithm
class Bresenham {
public:
  /// Make dy steps per dx ticks.
  /** Both dx and dy need to be non-negative integers, and dy <= dx.
      offset is used to control when first step arrives. Range 0-2,
      where 0 is on first tick, 2 on last tick and 1 inbetween.
   */
  Bresenham(int dy = 0, int dx = 0, int offset = 0) 
    : error(dx*offset - dy*2)
    , dxy((dx - dy)*2)
    , dy(dy*2)
  {
  }

  // Move one step in x direction.
  // Returns true if a step in y direction should be taken
  inline bool tick() {
    if (error < 0) {
      error += dxy;
      return true;
    }
    else {
      error -= dy;
      return false;
    }
  }

  inline bool is_zero_slope() {
    return dxy == 0;
  }    
private:
  int error, dxy, dy;
};

#endif
