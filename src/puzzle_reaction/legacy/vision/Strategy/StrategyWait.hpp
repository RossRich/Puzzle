#if !defined(_STRATEGY_WAIT_H_)
#define _STRATEGY_WAIT_H_

#include "Strategy.hpp"
<<<<<<< HEAD:src/puzzle_reaction/include/puzzle_reaction/vision/Strategy/StrategyWait.hpp
#include "../Utils/Utils.hpp"
=======
>>>>>>> puzzle_reaction:src/puzzle_reaction/legacy/vision/Strategy/StrategyWait.hpp
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
<<<<<<< HEAD:src/puzzle_reaction/include/puzzle_reaction/vision/Strategy/StrategyWait.hpp
  StrategyWait(BallTrackingRos *context) : _context(context) { _timer = ros::Time::now(); }
=======
  StrategyWait() {
    _timer = ros::Time::now();
  }
>>>>>>> puzzle_reaction:src/puzzle_reaction/legacy/vision/Strategy/StrategyWait.hpp

  ~StrategyWait() { std::cout << "Delete strategy\n"; }

  void execute() override;
};

#endif // _STRATEGY_WAIT_H_
