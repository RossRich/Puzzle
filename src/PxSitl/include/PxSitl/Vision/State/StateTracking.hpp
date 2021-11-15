#if !defined(_VISION_STATE_TRACKING_H_)
#define _VISION_STATE_TRACKING_H_

#include "State.hpp"

class StateTracking : public State {
private:
public:
  StateTracking(BallTrackingRos *context) : State(context) {}
  ~StateTracking() { std::cout << "Delete state\n"; }

  void tracking() override;
  void wait() override;
};

#endif // _VISION_STATE_TRACKING_H_
