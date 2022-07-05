#if !defined(_STATE_CONTEXT_H_)
#define _STATE_CONTEXT_H_

#include "State.hpp"

class StateContext {
protected:
  State *_state = nullptr;

public:
  StateContext() {}
  virtual ~StateContext();
  virtual void setState(State *state);
  virtual void tracking();
  virtual void wait();
  virtual void init();
  virtual void loop();
};

#endif // _STATE_CONTEXT_H_
