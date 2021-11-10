#if !defined(_VISION_STATE_WAIT_H_)
#define _VISION_STATE_WAIT_H_

#include "State.hpp"

class StateWait: public State
{
private:
  
public:
  StateWait(BallTrackingRos *context): State(context) {}
  ~StateWait() {
    std::cout << "Delete state\n";
  }

  void tracking() override;
  void wait() override;
};

#endif // _VISION_STATE_WAIT_H_
