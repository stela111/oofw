#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
struct Steps {
};

class Planner {
 public:
  Planner(unsigned queue_size);

  /**
     Add a move to plan.
   */
  void plan_move(Steps steps,
		 float max_change_speed_sqr,
		 float nominal_speed_sqr,
		 float max_entry_speed_sqr);

  struct Move {
    Steps steps; // Steps per axis
    float entry_speed_sqr; // Planned entry speed (squared)
  };

  bool check_full_buffer() const;
  const Move* get_current_move() const;
  float get_current_exit_speed_sqr() const;
  void next_move();
 private:
  void recalculate();
  std::size_t next_block_index(std::size_t block_index) const;
  std::size_t prev_block_index(std::size_t block_index) const;

  /**
   Data needed for planner for each linear block of motion.
   All speeds in (mm/min)^2 to simplify calculations.
   (Constant acceleration over distance s gives final speed v^2 = v0^2 + 2as)
   */
  struct PlanBlock : public Move {
    float nominal_speed_sqr; // Requested speed
    float max_entry_speed_sqr; // Max allowed entry speed
    float max_change_speed_sqr; // Max possible speed change in this block (2as term)
  };

  std::vector<PlanBlock> block_buffer;
  std::size_t block_buffer_tail;     // Index of the block to process now
  std::size_t block_buffer_head;     // Index of the next block to be pushed
  std::size_t next_buffer_head;      // Index of the next buffer head
  std::size_t block_buffer_planned;  // Index of the optimally planned block
};

#endif
