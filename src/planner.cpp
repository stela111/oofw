#include "planner.h"


Planner::Planner(unsigned queue_size, unsigned axes)
  : block_buffer(queue_size)
  , block_buffer_tail(0)
  , block_buffer_head(0)
  , next_buffer_head(1)
  , block_buffer_planned(0)
{
  for (auto &block : block_buffer) {
    block.steps.resize(axes);
  }
}


bool Planner::is_buffer_full() const
{
  return (block_buffer_tail == next_buffer_head);
}


const std::vector<int>* Planner::get_current_steps() const
{
  if (block_buffer_head == block_buffer_tail) {
    // Buffer empty  
    return nullptr;
  }
  return &block_buffer[block_buffer_tail].steps;
}


float Planner::get_current_entry_speed_sqr() const
{
  return block_buffer[block_buffer_tail].entry_speed_sqr;
}


float Planner::get_current_speed_sqr() const
{
  return block_buffer[block_buffer_tail].nominal_speed_sqr;
}


float Planner::get_current_exit_speed_sqr() const
{
  std::size_t block_index = next_block_index(block_buffer_tail);
  if (block_index == block_buffer_head) {
    // No next block
    return 0.0;
  }
  return block_buffer[block_index].entry_speed_sqr; 
}


void Planner::next_move()
{
  if (block_buffer_head != block_buffer_tail) { // Discard non-empty buffer.
    std::size_t block_index = next_block_index( block_buffer_tail );
    // Push block_buffer_planned pointer, if encountered.
    if (block_buffer_tail == block_buffer_planned) {
      block_buffer_planned = block_index; 
    }
    block_buffer_tail = block_index;
  }
}


void Planner::plan_move(const std::vector<int>& steps,
			float max_change_speed_sqr,
			float nominal_speed_sqr,
			float max_entry_speed_sqr)
{
  PlanBlock *block = &block_buffer[block_buffer_head];
  block->steps = steps;
  block->entry_speed_sqr = 0;
  block->nominal_speed_sqr = nominal_speed_sqr;
  block->max_change_speed_sqr = max_change_speed_sqr;

  if (block_buffer_head == block_buffer_tail) {
    block->max_entry_speed_sqr = 0;
  }
  else {
    // Not first block, compute entry speed
    float prev_nominal_speed_sqr = 
      block_buffer[prev_block_index(block_buffer_head)].nominal_speed_sqr;
    block->max_entry_speed_sqr = std::min(std::min(max_entry_speed_sqr,
						   nominal_speed_sqr),
					  prev_nominal_speed_sqr);
  }
  
  block_buffer_head = next_buffer_head;  
  next_buffer_head = next_block_index(block_buffer_head);
  
  // Finish up by recalculating the plan with the new block.
  recalculate();
}


std::size_t Planner::next_block_index(std::size_t block_index) const
{
  block_index++;
  if (block_index == block_buffer.size()) {
    block_index = 0;
  }
  return block_index;
}


// Returns the index of the previous block in the ring buffer
std::size_t Planner::prev_block_index(std::size_t block_index) const
{
  if (block_index == 0) {
    block_index = block_buffer.size();
  }
  block_index--;
  return block_index;
}


void Planner::recalculate() 
{   
  // Initialize block index to the last block in the planner buffer.
  std::size_t block_index = prev_block_index(block_buffer_head);
        
  // Bail. Can't do anything with one only one plan-able block.
  if (block_index == block_buffer_planned) {
    return;
  }

  // Reverse Pass: Coarsely maximize all possible deceleration curves back-planning from the last
  // block in buffer. Cease planning when the last optimal planned or tail pointer is reached.
  // NOTE: Forward pass will later refine and correct the reverse pass to create an optimal plan.
  float entry_speed_sqr;
  PlanBlock *next;
  PlanBlock *current = &block_buffer[block_index];

  // Calculate maximum entry speed for last block in buffer, where the exit speed is always zero.
  current->entry_speed_sqr = std::min(current->max_entry_speed_sqr,
				      current->max_change_speed_sqr);
  
  block_index = prev_block_index(block_index);
  if (block_index == block_buffer_planned) {
    // Only two plannable blocks in buffer. Reverse pass complete.
    // Check if the first block is the tail. If so, notify stepper to update its current parameters.
    if (block_index == block_buffer_tail) {
      //      stepper->update_plan_block_parameters();
    }
  }
  else { // Three or more plan-able blocks
    while (block_index != block_buffer_planned) { 
      next = current;
      current = &block_buffer[block_index];
      block_index = prev_block_index(block_index);

      // Check if next block is the tail block(=planned block). If so, update current stepper parameters.
      if (block_index == block_buffer_tail) {
	//	stepper->update_plan_block_parameters();
      } 

      // Compute maximum entry speed decelerating over the current block from its exit speed.
      if (current->entry_speed_sqr != current->max_entry_speed_sqr) {
        entry_speed_sqr = next->entry_speed_sqr + current->max_change_speed_sqr;
        if (entry_speed_sqr < current->max_entry_speed_sqr) {
          current->entry_speed_sqr = entry_speed_sqr;
        }
	else {
          current->entry_speed_sqr = current->max_entry_speed_sqr;
        }
      }
    }
  }    

  // Forward Pass: Forward plan the acceleration curve from the planned pointer onward.
  // Also scans for optimal plan breakpoints and appropriately updates the planned pointer.
  next = &block_buffer[block_buffer_planned]; // Begin at buffer planned pointer
  block_index = next_block_index(block_buffer_planned); 
  while (block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    
    // Any acceleration detected in the forward pass automatically moves the optimal planned
    // pointer forward, since everything before this is all optimal. In other words, nothing
    // can improve the plan from the buffer tail to the planned pointer by logic.
    if (current->entry_speed_sqr < next->entry_speed_sqr) {
      entry_speed_sqr = current->entry_speed_sqr + current->max_change_speed_sqr;
      // If true, current block is full-acceleration and we can move the planned pointer forward.
      if (entry_speed_sqr < next->entry_speed_sqr) {
        next->entry_speed_sqr = entry_speed_sqr; // Always <= max_entry_speed_sqr. Backward pass sets this.
        block_buffer_planned = block_index; // Set optimal plan pointer.
      }
    }
    
    // Any block set at its maximum entry speed also creates an optimal plan up to this
    // point in the buffer. When the plan is bracketed by either the beginning of the
    // buffer and a maximum entry speed or two maximum entry speeds, every block in between
    // cannot logically be further improved. Hence, we don't have to recompute them anymore.
    if (next->entry_speed_sqr == next->max_entry_speed_sqr) {
      block_buffer_planned = block_index;
    }
    block_index = next_block_index( block_index );
  } 
}
