#if !defined(_STRATEGY_WAIT_H_)
#define _STRATEGY_WAIT_H_

#include "Strategy.hpp"
#include "Utils/Utils.hpp"
#include "ros/ros.h"

class BallTrackingRos;

class StrategyWait: public Strategy
{
private:
  ros::Time _timer;
  ros::Duration _timeOut = ros::Duration(5);
  BallTrackingRos *_context = nullptr;

public:
  StrategyWait(BallTrackingRos *context):_context(context) {
    _timer = ros::Time::now();
  }

  ~StrategyWait() {
    std::cout << "Delete strategy\n";
  }

  void execute() override;
};

#endif // _STRATEGY_WAIT_H_
