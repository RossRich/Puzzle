#if !defined(_TRACKING_STRATEGY_H_)
#define _TRACKING_STRATEGY_H_

#include "../BallTracking.hpp"
#include "Strategy.hpp"

class TrakcingStrategy : public Strategy {
private:
  const char *_winName = "Tracking";

  cv::Mat _frame;
  VideoHandler &_vh;

public:
  TrakcingStrategy(VideoHandler &vh) : _vh(vh) {
    cv::namedWindow(_winName, cv::WINDOW_AUTOSIZE);

    std::cout << "Tracking mode ready\n";
  }
  ~TrakcingStrategy() { cv::destroyWindow(_winName); }

  void execute() override {
    _vh >> _frame;

    if(_frame.empty()){
      std::cerr << "Frame is empty\n";
      return;
    }

    cv::imshow(_winName, _frame);
    cv::waitKey(1);
  }
};

#endif // _TRACKING_STRATEGY_H_
