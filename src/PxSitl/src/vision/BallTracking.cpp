#include "../../include/PxSitl/vision/BallTracking.hpp"



BallTracking::BallTracking(uint16_t width, uint16_t height, cv::Vec<cv::Scalar_<uint8_t>, 2> threshold) {
  _imSize = Size2i(width, height);
  _threshold = {0};
}

BallTracking::~BallTracking() {}

void BallTracking::operator=(const BallTracking &bt) {
  _imSize = bt._imSize;
  _threshold = bt._threshold;
}

Point2i BallTracking::process(Mat &color) {

  Mat frame = color.clone();
  Mat mask;

  cv::resize(frame, frame, _imSize);

  GaussianBlur(frame, mask, cv::Size2i(11, 11), 0);
  cv::cvtColor(frame, mask, cv::COLOR_RGB2HSV);
  cv::inRange(mask, cv::Scalar(_threshold[0][0], _threshold[0][1], _threshold[0][2]),
              cv::Scalar(_threshold[1][0], _threshold[1][1], _threshold[1][2]), mask);

  cv::erode(mask, mask, Mat(), cv::Point2i(), 5);
  cv::dilate(mask, mask, Mat(), cv::Point2i(), 6);

  cv::findContours(mask.clone(), _cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  float radius = 0.0;
  cv::Point2f center;
  if (_cnts.size() > 0) {
    std::vector<cv::Point> cnt0 = _cnts[0];
    int maxSiza = _cnts[0].size();
    for (auto c : _cnts) {
      if (c.size() > maxSiza) {
        cnt0 = c;
        maxSiza = c.size();
      }
    }

    cv::minEnclosingCircle(cnt0, center, radius);

    /* cv::circle(frame, center, radius + 7.0, cv::Scalar(_threshold[1]), 1, cv::LINE_4);
    cv::circle(frame, center, 3, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
    cv::Point2f textPos = {center.x + radius + 15.0f, center.y + radius + 15.0f};
    cv::putText(frame, _info.str(), textPos, cv::FONT_HERSHEY_SIMPLEX, .6, cv::Scalar::all(0), 2); */
  }

  return {static_cast<uint16_t>(center.x), static_cast<uint16_t>(center.y)};
}