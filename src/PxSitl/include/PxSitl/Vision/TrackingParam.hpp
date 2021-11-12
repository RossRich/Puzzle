#if !defined(_TRACKING_PARAM_H_)
#define _TRACKING_PARAM_H_

<<<<<<< HEAD
#include "VideoHandler.hpp"
#include "Utils/thresholdtype.hpp"
=======
#include "../Utils/thresholdtype.hpp"
#include "VideoHandler.hpp"
>>>>>>> 8ad21a8c05e4c999f491118b7574d61db3ade669

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class TrackingParam {

private:
  const char *_winName = "Main";
  static const char *_confFile;
  uint8_t _roiPointsNum = 0;
  uint16_t _counter = 100;
  std::vector<cv::Point2i> _roiPoints;
  cv::Point2i _cursor;
  threshold_t _threshold;
  cv::Mat _frame;
  VideoHandler &_vh;

public:
  TrackingParam(VideoHandler &vh);
  ~TrackingParam();

  void onMouseCallback(int ev, int x, int y, int flag);
  bool newThreshold(cv::Mat &mask);
  bool getThreshold(threshold_t &td);
  void maskFormGUI(cv::Mat &mask);
  void getMask();
};

#endif // _TRACKING_PARAM_H_
