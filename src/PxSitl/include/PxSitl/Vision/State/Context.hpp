#if !defined(_STATE_CONTEXT_H_)
#define _STATE_CONTEXT_H_

#include "State.hpp"

class Context {
private:
  State *_state = nullptr;

public:
  Context(State *state) { transitionTo(state); }
  virtual ~Context() {
    delete _state;
    _state = nullptr;
  }

  virtual void toSetupState() = 0;
  virtual void toTrackingState() = 0;

protected:
  void transitionTo(State *state) {
    if (state != nullptr && state != _state) {
      delete _state;
      _state = state;
      _state->setContext(this);
      std::cout << "Transition to next state\n";
    }
  }
};

#endif // _STATE_CONTEXT_H_
