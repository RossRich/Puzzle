#include "../../include/PxSitl/vision/BallTracking.hpp"

BallTracking::BallTracking(uint16_t width, uint16_t height, cv::Vec<cv::Scalar_<uint8_t>, 2> threshold) {
  _imSize = cv::Size2i(width, height);
  _threshold = threshold;
}

BallTracking::~BallTracking() {}

/* void BallTracking::operator=(const BallTracking &bt) {
  _imSize = bt._imSize;
  _threshold = bt._threshold;
} */

void BallTracking::process(cv::Mat &color, cv::Mat &maskt, cv::Point2i *center, uint16_t *radius) {

  cv::Mat frame = color.clone();
  cv::Mat mask(_imSize, CV_8UC1, cv::Scalar::all(0));

  GaussianBlur(frame, frame, cv::Size2i(9, 9), 0);
  cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);

  cv::inRange(frame, cv::Scalar(_threshold[0]), cv::Scalar(_threshold[1]), mask);

  // cv::erode(mask, mask, cv::Mat(), cv::Point2i(), 5);
  // cv::dilate(mask, mask, cv::Mat(), cv::Point2i(), 6);

  maskt = mask.clone();

  cv::findContours(mask.clone(), _cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  float R = 0.0;
  cv::Point2f c;
  if (_cnts.size() > 0) {
    std::vector<cv::Point> cnt0 = _cnts[0];
    int maxSiza = _cnts[0].size();
    for (auto c : _cnts) {
      if (c.size() > maxSiza) {
        cnt0 = c;
        maxSiza = c.size();
      }
    }

    cv::minEnclosingCircle(cnt0, c, R);

    if (center != nullptr)
      *center = c;

    if (radius != nullptr)
      *radius = static_cast<uint16_t>(R);

    /* cv::circle(frame, center, radius + 7.0, cv::Scalar(_threshold[1]), 1, cv::LINE_4);
    cv::circle(frame, center, 3, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
    cv::Point2f textPos = {center.x + radius + 15.0f, center.y + radius + 15.0f};
    cv::putText(frame, _info.str(), textPos, cv::FONT_HERSHEY_SIMPLEX, .6, cv::Scalar::all(0), 2); */
  }
}
