#include "puzzle_reaction/vision/BallTracking.hpp"

BallTracking::BallTracking(uint16_t width, uint16_t height, cv::Vec<cv::Scalar_<uint8_t>, 2> threshold) {
  _imSize = cv::Size2i(width, height);
  _threshold = threshold;
}

void BallTracking::process(cv::Mat &color, cv::Mat &mask, cv::Point2i *center, uint16_t *radius) {

  cv::Mat frame = color.clone();
  cv::Mat colorMask(_imSize, CV_8UC1, cv::Scalar::all(0));

  GaussianBlur(frame, frame, cv::Size2i(9, 9), 0);
  cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);

  cv::inRange(frame, cv::Scalar(_threshold[0]), cv::Scalar(_threshold[1]), colorMask);

  cv::erode(colorMask, colorMask, cv::Mat(), cv::Point2i(), 2);
  cv::dilate(colorMask, colorMask, cv::Mat(), cv::Point2i(), 1);

  mask = colorMask.clone();

  cv::findContours(colorMask, _cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  
  if (_cnts.size() > 0) {
    std::vector<cv::Point> bestCnt = _cnts[0];
    int maxSiza = _cnts[0].size();
    for (auto cnt : _cnts) {
      if(cnt.size() < 20) ///< TODO: to params
        continue;

      if (cnt.size() > maxSiza) {
        bestCnt = cnt;
        maxSiza = cnt.size();
      }
    }

    float circleRadius = 0.0;
    cv::Point2f circleCenter;
    cv::minEnclosingCircle(bestCnt, circleCenter, circleRadius);

    if (center != nullptr)
      *center = circleCenter;

    if (radius != nullptr)
      *radius = static_cast<uint16_t>(std::round(circleRadius));

    /* cv::circle(frame, center, radius + 7.0, cv::Scalar(_threshold[1]), 1, cv::LINE_4);
    cv::circle(frame, center, 3, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
    cv::Point2f textPos = {center.x + radius + 15.0f, center.y + radius + 15.0f};
    cv::putText(frame, _info.str(), textPos, cv::FONT_HERSHEY_SIMPLEX, .6, cv::Scalar::all(0), 2); */
  }
}
