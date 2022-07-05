#include "../../include/PxSitl/vision/StateContext.hpp"

StateContext::~StateContext() {
  delete _state;
}

void StateContext::setState(State *state) {
  if (state != nullptr) {
    delete _state;
    _state = state;
    std::cout << "Transition to new state " << _state->toString() << std::endl;
  }
}

void StateContext::tracking() {
  _state->tracking();
}

void StateContext::wait() {
  _state->wait();
}

void StateContext::loop() {
  _state->execute();
}
