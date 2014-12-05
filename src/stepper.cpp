#include "stepper.h"

Stepper::Stepper(PinIo *io, Pins pins)
  : io_(io)
  , pins_(pins)
  , position_(0)
  , state_(State::DISABLED)
  , direction_(true)
  , stop_on_endstop_(true)
{
  io_->clear(pins_.enable);
  io_->clear(pins_.step);
  io_->clear(pins_.dir);
}

Stepper::State Stepper::state() const {
  return state_;
}

void Stepper::enable() {
  io_->set(pins_.enable);
  state_ = State::ACTIVE;
}

void Stepper::disable() {
  io_->clear(pins_.enable);
  state_ = State::DISABLED;
}

void Stepper::step() {
  if (state_ == State::ACTIVE) {
    io_->set(pins_.step);
    position_ += direction_?1:-1;
    state_ = State::STEPPING;
  }
}

void Stepper::unstep() {
  if (state_ == State::STEPPING) {
    io_->clear(pins_.step);
    if (stop_on_endstop_ && io_->get(pins_.endstop)) {
      state_ = State::STOPPED;
    }
    else {
      state_ = State::ACTIVE;
    }
  }
}

void Stepper::set_direction(bool direction) {
  if (direction) {
    io_->set(pins_.dir);
  }
  else {
    io_->clear(pins_.dir);
  }
  direction_ = direction;
}

bool Stepper::is_endstop_active() const {
  return io_->get(pins_.endstop);
}

void Stepper::stop_on_endstop(bool stop) {
  stop_on_endstop_ = stop;
}
