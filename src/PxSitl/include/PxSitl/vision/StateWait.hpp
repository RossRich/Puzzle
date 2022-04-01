#if !defined(_VISION_STATE_WAIT_H_)
#define _VISION_STATE_WAIT_H_

#include "State.hpp"

class StateWait : public State {
private:
public:
  StateWait() : State("Wait") {}
  ~StateWait() {
    std::cout << "Delete state wait\n";
  }

  // void tracking(BallTrackingRos *) override;
  void wait(BallTrackingRos *context) override;
  void init(BallTrackingRos *context) override;
  void execute() override;
};

#endif // _VISION_STATE_WAIT_H_
