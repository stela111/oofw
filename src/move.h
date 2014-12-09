#ifndef MOVE_H
#define MOVE_H

/// Info about a move
struct Move {
  std::vector<int> steps;
  float length;
  float speed;
  float acceleration;
};

#endif
