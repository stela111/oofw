#include "planner.h"


Planner::Planner(unsigned queue_size)
  : block_buffer(queue_size)
  , block_buffer_tail(0)
  , block_buffer_head(0)
  , next_buffer_head(1)
  , block_buffer_planned(0)
{
}


bool Planner::check_full_buffer() const
{
  return (block_buffer_tail == next_buffer_head);
}


const Planner::Move *Planner::get_current_move() const
{
  if (block_buffer_head == block_buffer_tail) {
    // Buffer empty  
    return(0);
  }
  return(&block_buffer[block_buffer_tail]);
}


float Planner::get_current_exit_speed_sqr() const
{
  std::size_t block_index = next_block_index(block_buffer_tail);
  if (block_index == block_buffer_head) {
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


void Planner::plan_move(Steps steps,
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

/*                            PLANNER SPEED DEFINITION                                              
                                     +--------+   <- current->nominal_speed
                                    /          \                                
         current->entry_speed ->   +            \                               
                                   |             + <- next->entry_speed (aka exit speed)
                                   +-------------+                              
                                       time -->                      
                                                  
  Recalculates the motion plan according to the following basic guidelines:
  
    1. Go over every feasible block sequentially in reverse order and calculate the junction speeds
        (i.e. current->entry_speed) such that:
      a. No junction speed exceeds the pre-computed maximum junction speed limit or nominal speeds of 
         neighboring blocks.
      b. A block entry speed cannot exceed one reverse-computed from its exit speed (next->entry_speed)
         with a maximum allowable deceleration over the block travel distance.
      c. The last (or newest appended) block is planned from a complete stop (an exit speed of zero).
    2. Go over every block in chronological (forward) order and dial down junction speed values if 
      a. The exit speed exceeds the one forward-computed from its entry speed with the maximum allowable
         acceleration over the block travel distance.
  
  When these stages are complete, the planner will have maximized the velocity profiles throughout the all
  of the planner blocks, where every block is operating at its maximum allowable acceleration limits. In 
  other words, for all of the blocks in the planner, the plan is optimal and no further speed improvements
  are possible. If a new block is added to the buffer, the plan is recomputed according to the said 
  guidelines for a new optimal plan.
  
  To increase computational efficiency of these guidelines, a set of planner block pointers have been
  created to indicate stop-compute points for when the planner guidelines cannot logically make any further
  changes or improvements to the plan when in normal operation and new blocks are streamed and added to the
  planner buffer. For example, if a subset of sequential blocks in the planner have been planned and are 
  bracketed by junction velocities at their maximums (or by the first planner block as well), no new block
  added to the planner buffer will alter the velocity profiles within them. So we no longer have to compute
  them. Or, if a set of sequential blocks from the first block in the planner (or a optimal stop-compute
  point) are all accelerating, they are all optimal and can not be altered by a new block added to the
  planner buffer, as this will only further increase the plan speed to chronological blocks until a maximum
  junction velocity is reached. However, if the operational conditions of the plan changes from infrequently
  used feed holds or feedrate overrides, the stop-compute pointers will be reset and the entire plan is  
  recomputed as stated in the general guidelines.
  
  Planner buffer index mapping:
  - block_buffer_tail: Points to the beginning of the planner buffer. First to be executed or being executed. 
  - block_buffer_head: Points to the buffer block after the last block in the buffer. Used to indicate whether
      the buffer is full or empty. As described for standard ring buffers, this block is always empty.
  - next_buffer_head: Points to next planner buffer block after the buffer head block. When equal to the 
      buffer tail, this indicates the buffer is full.
  - block_buffer_planned: Points to the first buffer block after the last optimally planned block for normal
      streaming operating conditions. Use for planning optimizations by avoiding recomputing parts of the 
      planner buffer that don't change with the addition of a new block, as describe above. In addition, 
      this block can never be less than block_buffer_tail and will always be pushed forward and maintain 
      this requirement when encountered by the plan_discard_current_block() routine during a cycle.
  
  NOTE: Since the planner only computes on what's in the planner buffer, some motions with lots of short 
  line segments, like G2/3 arcs or complex curves, may seem to move slow. This is because there simply isn't
  enough combined distance traveled in the entire buffer to accelerate up to the nominal speed and then 
  decelerate to a complete stop at the end of the buffer, as stated by the guidelines. If this happens and
  becomes an annoyance, there are a few simple solutions: (1) Maximize the machine acceleration. The planner
  will be able to compute higher velocity profiles within the same combined distance. (2) Maximize line 
  motion(s) distance per block to a desired tolerance. The more combined distance the planner has to use,
  the faster it can go. (3) Maximize the planner buffer size. This also will increase the combined distance
  for the planner to compute over. It also increases the number of computations the planner has to perform
  to compute an optimal plan, so select carefully. The Arduino 328p memory is already maxed out, but future
  ARM versions should have enough memory and speed for look-ahead blocks numbering up to a hundred or more.

*/
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
