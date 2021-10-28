#if !defined(_BALL_TRACKING_H_)
#define _BALL_TRACKING_H_

#include "Utils.hpp"
#include <deque>
#include <iostream>
#include <librealsense2/rs.hpp>
// #include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <time.h>

using cv::Mat;
using cv::Point2i;
using cv::Size2i;

class BallTracking {

private:
  Size2i _imSize;
  cv::Vec<cv::Scalar_<uint8_t>, 2> _threshold;
  std::vector<std::vector<cv::Point2i>> _cnts;
  // std::stringstream _info;

  cv::Size2i _filterSize = cv::Size2i(3, 3);
  cv::Matx33f _dist2Meters = cv::Matx33d::all(0.001f);

  static const char *_confFile;

public:
  BallTracking() {}
  BallTracking(uint16_t width, uint16_t height, cv::Vec<cv::Scalar_<uint8_t>, 2> threshold);
  ~BallTracking();

  void operator=(const BallTracking &bt);

  Point2i process(Mat &color);
};

#endif // _BALL_TRACKING_H_
