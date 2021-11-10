#if !defined(_STRATEGY_WAIT_H_)
#define _STRATEGY_WAIT_H_

#include "BallTrackingRos.hpp"
#include "Strategy.hpp"
#include "Utils/Utils.hpp"
#include "ros/ros.h"

class StrategyWait: public Strategy
{
private:
  ros::Time _timer;
  ros::Duration _timeOut = ros::Duration(10);
  BallTrackingRos *_context = nullptr;

public:
  StrategyWait(BallTrackingRos *context):_context(context) {
    _timer = ros::Time::now();
  }

  ~StrategyWait() {
    std::cout << "Delete strategy\n";
  }

  void execute() override {

    if(ros::Duration(ros::Time::now() - _timer) >= _timeOut) {
      
      threshold_t thresh;
      if(Utils::readThresholds(_context->getConfFile(), thresh)) {
        ROS_INFO("Data available");
        _context->tracking();
      } else
        ROS_WARN("No data");
      
      _timer = ros::Time::now();
    }

  }
};

#endif // _STRATEGY_WAIT_H_
