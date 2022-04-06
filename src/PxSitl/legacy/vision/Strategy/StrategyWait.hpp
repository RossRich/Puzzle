#if !defined(_STRATEGY_WAIT_H_)
#define _STRATEGY_WAIT_H_

#include "Strategy.hpp"
#include "ros/ros.h"
#include "utils/Utils.hpp"

// class BallTrackingRos;

class StrategyWait : public Strategy {
private:
  ros::Time _timer;
  ros::Duration _timeOut = ros::Duration(5);
  // BallTrackingRos *_context = nullptr;
  std::string _confFile = "/workspaces/Puzzle/src/PxSitl/data/config.yaml";

public:
  StrategyWait() {
    _timer = ros::Time::now();
  }

  ~StrategyWait() {
    std::cout << "Delete strategy\n";
  }

  void execute() override;
};

#endif // _STRATEGY_WAIT_H_
