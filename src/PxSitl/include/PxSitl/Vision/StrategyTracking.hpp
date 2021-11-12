#if !defined(_STRATEGY_TRACKING_H_)
#define _STRATEGY_TRACKING_H_

#include <opencv2/highgui/highgui.hpp>
#include "BallTracking.hpp"
#include "Strategy.hpp"
#include "VideoHandler.hpp"
#include "Utils/thresholdtype.hpp"
#include "Utils/Utils.hpp"
#include "ros/ros.h"

class BaBallTrackingRos;

class StrategyTracking : public Strategy {
private:
  const char* _winName = "Tracking";
  BallTracking _bt;
  VideoHandler &_vh;
  cv::Mat _frame;
  BallTrackingRos *_context;
  

public:
  StrategyTracking(VideoHandler &vh, BallTrackingRos *context) : _vh(vh), _context(context) {
    cv::namedWindow(_winName, cv::WINDOW_AUTOSIZE);
  }

  ~StrategyTracking() {
    std::cout << "Delete strategy\n";
    cv::destroyWindow(_winName);
  }

  bool init();
  void execute() override;
};

#endif // _STRATEGY_TRACKING_H_
