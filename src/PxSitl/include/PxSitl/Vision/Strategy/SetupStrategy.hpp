#if !defined(_SETUP_STRATEGY_H_)
#define _SETUP_STRATEGY_H_

#include "../TrackingParam.hpp"
#include "Strategy.hpp"

class SetupStrategy : public Strategy {
private:
  const char *_winName = "Setup";

  cv::Mat _frame;
  TrackingParam _tp;
  VideoHandler &_vh;

public:
  SetupStrategy(VideoHandler &vh) : _vh(vh), _tp(vh) {
    cv::namedWindow(_winName, cv::WINDOW_AUTOSIZE);
    std::cout << "Setup mode ready\n";
  }

  ~SetupStrategy() {
    std::cout << "Delete Setup strategy\n";
    cv::destroyWindow(_winName);
  }

  void execute() override {
    _vh >> _frame;

    if (_frame.empty()) {
      std::cerr << "Frame is empty\n";
      return;
    }

    cv::imshow(_winName, _frame);
    char c = (uint8_t)cv::waitKey(1);

    if (c == 'q') {
      std::cout << "q" << std::endl;
    }
  }
};

#endif // _SETUP_STRATEGY_H_
