#ifndef PLANNER_H
#define PLANNER_H

#include <vector>

/// Temporary placeholder while thinking
struct Move {};

/// Plans a sequence of moves for maximum speed given constraints.
/**
   This planner is built around the formula: v^2 = v0^2 + 2as, for
   final speed v, initial speed v0, acceleration a and distance s.
   Each move is associated with constraints:
   - requested max speed
   - max entry speed
   - maximum term 2as

   For each added move, the planner algorithm optimizes the entry and exit
   speed for each move in the plan so that all constraints are met.
*/
class Planner {
 public:
  /// Create a planner capable of planning queue_size moves ahead
  Planner(unsigned queue_size, unsigned axes);

  /// Add a move to plan.
  void plan_move(const std::vector<int>& steps,
		 float max_change_speed_sqr,
		 float nominal_speed_sqr,
		 float max_entry_speed_sqr);

  /// Check if no moves can be added
  bool is_buffer_full() const;

  /// Returns current steps or nullptr if empty
  const std::vector<int>* get_current_steps() const;

  /// Get planned entry speed for current move
  float get_current_entry_speed_sqr() const;

  /// Get requested nominal speed for current move
  float get_current_speed_sqr() const;

  /// Get planned exit speed for current move
  float get_current_exit_speed_sqr() const;

  /// Discard current move
  void next_move();
 private:
  void recalculate();
  std::size_t next_block_index(std::size_t block_index) const;
  std::size_t prev_block_index(std::size_t block_index) const;

  /**
   Data needed for planner for each linear block of motion.
   */
  struct PlanBlock {
    std::vector<int> steps; // Planned number of steps per axis
    float entry_speed_sqr; // Planned entry speed (squared)
    float nominal_speed_sqr; // Requested speed (squared)
    float max_entry_speed_sqr; // Max allowed entry speed (squared)
    float max_change_speed_sqr; // Max possible speed change in this block (2as term)
  };

  std::vector<PlanBlock> block_buffer;
  std::size_t block_buffer_tail;     // Index of the block to process now
  std::size_t block_buffer_head;     // Index of the next block to be pushed
  std::size_t next_buffer_head;      // Index of the next buffer head
  std::size_t block_buffer_planned;  // Index of the optimally planned block
};

#endif
