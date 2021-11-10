#if !defined(_STRATEGY_TRACKING_H_)
#define _STRATEGY_TRACKING_H_

#include "BallTracking.hpp"
#include "Strategy.hpp"
#include "VideoHandler.hpp"
#include "Utils/thresholdtype.hpp"
#include "Utils/Utils.hpp"
#include "ros/ros.h"
#include "BallTrackingRos.hpp"

class StrategyTracking : public Strategy {
private:
  BallTracking _bt;
  VideoHandler &_vh;
  cv::Mat _frame;
  BallTrackingRos *_context;

public:
  StrategyTracking(VideoHandler &vh, BallTrackingRos *context) : _vh(vh), _context(context) {}

  ~StrategyTracking() {
    std::cout << "Delete strategy\n";
  }

  bool init() {
    threshold_t threshold;
    if(Utils::readThresholds(_context->getConfFile(), threshold))
      _bt = BallTracking(_vh.getWidth(), _vh.getHeight(), threshold);
    else
      return false;

    return true;
  }

  void execute() override {
    _vh >> _frame;

    if (_frame.empty()) {
      ROS_WARN("Frame is empty");
    }

    // _bt.process();
  }
};

#endif // _STRATEGY_TRACKING_H_
