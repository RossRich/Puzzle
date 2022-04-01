#if !defined(_TRACKING_PARAM_H_)
#define _TRACKING_PARAM_H_

#include "../utils/Utils.hpp"
#include "../utils/thresholdtype.hpp"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class TrackingParam {

private:
  const char *_confFile = "";
  threshold_t _threshold;
  cv::Mat _frame;
  cv::Mat _mask;

public:
  TrackingParam() {}
  TrackingParam(const char *confFile);
  ~TrackingParam() {}

  void setConfFile(const char *confFile) { _confFile = confFile; }

  const cv::Mat &getColor() { return _frame; }
  const cv::Mat &getMask() { return _mask; }
  const char *getConfFile() { return _confFile; }

  bool getThreshold(threshold_t &td);
  bool newThreshold(threshold_t &thresh);
  bool maskFormPoints(cv::Mat &frame, std::vector<cv::Point2i> &points);
};

#endif // _TRACKING_PARAM_H_
