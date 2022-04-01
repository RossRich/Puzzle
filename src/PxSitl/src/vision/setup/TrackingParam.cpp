#include "../../../include/PxSitl/vision/setup/TrackingParam.hpp"

TrackingParam::TrackingParam(const char *confFile) : _confFile(confFile) {}

bool TrackingParam::getThreshold(threshold_t &td) {
  if (_confFile == "")
    return false;

  return Utils::readThresholds(_confFile, td);
}

bool TrackingParam::maskFormPoints(cv::Mat &frame, std::vector<cv::Point2i> &points) {
  uint8_t roiPoints = points.size();

  if (roiPoints == 4 && !frame.empty()) {

    _mask.create(frame.size(), CV_8UC1);
    _mask = 0;

    try {
      cv::fillConvexPoly(_mask, points, cv::Scalar::all(255));
    } catch (cv::Exception &e) {
      std::cerr << e.what() << std::endl;
      _mask = 0;
      return false;
    }

    frame.copyTo(_frame);
  } else
    return false;

  return true;
}

bool TrackingParam::newThreshold(threshold_t &thresh) {
  if (_mask.empty() || _frame.empty())
    return false;

  cv::Mat roi;
  _frame.copyTo(roi, _mask);
  cv::GaussianBlur(roi, roi, cv::Size2i(9,9), 1);
  cv::cvtColor(roi, roi, cv::COLOR_BGR2HSV);

  cv::Mat channels[3];
  cv::split(roi, channels);

  double min, max;
  int idxMin, idxMax;

  cv::Scalar_<uint8_t> maxThresh = cv::Scalar::all(0);
  cv::Scalar_<uint8_t> minThresh = maxThresh;

  for (uint8_t i = 0; i < 3; i++) {
    cv::minMaxIdx(channels[i], &min, &max, &idxMin, &idxMax, _mask);
    std::cout << "In layer " << int(i) << " min val: " << min << " max val: " << max << std::endl;
    maxThresh[i] = static_cast<uint8_t>(max);
    minThresh[i] = static_cast<uint8_t>(min);
  }

  _threshold[0] = minThresh;
  _threshold[1] = maxThresh;

  std::cout << "New threshold\n";
  std::cout << "Min: " << _threshold[0] << std::endl;
  std::cout << "Max: " << _threshold[1] << std::endl;

  if (!Utils::writeThreshold(_confFile, _threshold))
    return false;

  return true;
}
