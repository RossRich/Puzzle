#if !defined(_BALL_TRACKING_H_)
#define _BALL_TRACKING_H_

#include "utils/thresholdtype.hpp"
#include <deque>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class BallTracking {

private:
  cv::Size2i _imSize;
  threshold_t _threshold;
  std::vector<std::vector<cv::Point2i>> _cnts;
  // std::stringstream _info;

  cv::Size2i _filterSize = cv::Size2i(3, 3);
  cv::Matx33f _dist2Meters = cv::Matx33d::all(0.001f);

  // static const char *_confFile;

public:
  BallTracking(uint16_t width, uint16_t height, cv::Vec<cv::Scalar_<uint8_t>, 2> threshold);
  ~BallTracking();

  // void operator=(const BallTracking &bt);
  void process(cv::Mat &color, cv::Mat &mask, cv::Point2i *center = nullptr, uint16_t *radius = nullptr);

  inline void setThreshold(const threshold_t th) { _threshold = th; }
};

#endif // _BALL_TRACKING_H_
