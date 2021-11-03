#if !defined(_TRACKING_PARAM_H_)
#define _TRACKING_PARAM_H_

#include "VideoHandler.hpp"
#include "../Utils/thresholdtype.hpp"

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using cv::Mat;
using std::cerr;
using std::cout;
using std::endl;

class TrackingParam {
private:
  const char *_winName = "Main";
  static const char *_confFile;
  uint8_t _roiPointsNum = 0;
  std::vector<cv::Point2i> _roiPoints;
  cv::Point2i _cursor;
  threshold_t _threshold;
  Mat _frame;
  VideoHandler &_vh;

public:
  TrackingParam(VideoHandler &vh);
  ~TrackingParam();

  void onMouseCallback(int ev, int x, int y, int flag);
  bool newThreshold(Mat &mask);
  bool getThreshold(threshold_t &td);
  void maskFormGUI(Mat &mask);
  void getMask();
};

#endif // _TRACKING_PARAM_H_
