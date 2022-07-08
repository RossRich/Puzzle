#if !defined(_VISION_STATE_INIT_H_)
#define _VISION_STATE_INIT_H_

#include "State.hpp"

class StateInit : public State {
private:
public:
  StateInit(BallTrackingRos &context) : State(context, "Init") {}
  ~StateInit() {
    std::cout << "Delete state init\n";
  }

  void tracking() override {}
  void wait() override;
  void init() override;
  void execute() override;
};

#endif // _VISION_STATE_INIT_H_
