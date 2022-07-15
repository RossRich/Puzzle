#if !defined(_VISION_STATE_WAIT_H_)
#define _VISION_STATE_WAIT_H_

#include "State.hpp"
#include <ros/ros.h>

class StateWait : public State {
private:
  ros::Time _waitTimer;
public:
  StateWait(BallTrackingRos &context) : State(context, "Wait"), _waitTimer(ros::Time::now()) {}
  ~StateWait() {
    std::cout << "Delete state wait\n";
  }

  void tracking() override;
  void wait() override {}
  void execute() override;
};

#endif // _VISION_STATE_WAIT_H_
