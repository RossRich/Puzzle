#if !defined(_VISION_STATE_INIT_H_)
#define _VISION_STATE_INIT_H_

#include "State.hpp"

class StateInit : public State {
private:
public:
  StateInit() : State("Init") {}
  ~StateInit() {
    std::cout << "Delete state init\n";
  }

  // void tracking(BallTrackingRos *context) override;
  void wait(BallTrackingRos *context) override;
  void init(BallTrackingRos *context) override;
  void execute() override;
};

#endif // _VISION_STATE_INIT_H_
