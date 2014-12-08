#ifndef MOVE_H
#define MOVE_H

/// Info about a move
struct Move {
  std::vector<int> steps;
  unsigned events;
  float speed;
  float acceleration;
};

#endif
