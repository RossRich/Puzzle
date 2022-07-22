#include "puzzle_reaction/vision/BallTracking.hpp"

BallTracking::BallTracking(uint16_t width, uint16_t height, cv::Vec<cv::Scalar_<uint8_t>, 2> threshold) {
  _imSize = cv::Size2i(width, height);
  _threshold = threshold;
  _gpuColorImg = cv::cuda::GpuMat(_imSize, CV_8UC3, cv::Scalar::all(0));
  _gpuMaskImg = cv::cuda::GpuMat(_imSize, CV_8UC1, cv::Scalar::all(0));
  _gaussFilter = cv::cuda::createGaussianFilter(_gpuColorImg.type(), -1, cv::Size2i(5, 5), 0);
  _closeFilterKernel = cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, cv::Size(5, 5));
  _closeFilter = cv::cuda::createMorphologyFilter(cv::MORPH_CLOSE, _gpuMaskImg.type(), _closeFilterKernel, cv::Point(-1, -1), 2);
  _erodeFilter = cv::cuda::createMorphologyFilter(cv::MORPH_ERODE, _gpuMaskImg.type(), _closeFilterKernel, cv::Point(), 2);
  _dilateFilter = cv::cuda::createMorphologyFilter(cv::MORPH_DILATE, _gpuMaskImg.type(), _closeFilterKernel, cv::Point(), 1);
}

void BallTracking::process(cv::Mat &color, cv::Mat &mask, cv::Point2f *center, uint16_t *radius) {
  ros::Time timer = ros::Time::now();
  // cv::Mat frame = color.clone();
  _gpuColorImg.upload(color);
  // cv::cuda::Stream gpuStream;
  // ROS_INFO_STREAM("!!!" <<_imSize);
  // cv::Mat colorMask(_imSize, CV_8UC1, cv::Scalar::all(0));
  // ROS_INFO("!!!");
  // cv::medianBlur(frame, frame, 5);

  // GaussianBlur(frame, frame, cv::Size2i(5, 5), 0);
  _gaussFilter->apply(_gpuColorImg, _gpuColorImg);
  // cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);
  cv::cuda::cvtColor(_gpuColorImg, _gpuColorImg, cv::COLOR_BGR2HSV, 0);
  // cv::inRange(frame, cv::Scalar(_threshold[0]), cv::Scalar(_threshold[1]), colorMask);
  cv::cuda::inRange(_gpuColorImg, cv::Scalar(_threshold[0]), cv::Scalar(_threshold[1]), _gpuMaskImg);
  // _erodeFilter->apply(_gpuMaskImg, _gpuMaskImg);
  // _dilateFilter->apply(_gpuMaskImg, _gpuMaskImg);
  // cv::Mat kernel = cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, cv::Size(5,5));
  // cv::morphologyEx(colorMask, colorMask, cv::MORPH_CLOSE, kernel, cv::Point(-1,-1), 5);
  // cv::dilate(colorMask, colorMask, cv::Mat(), cv::Point2i(), 5);
  // _closeFilter->apply(_gpuMaskImg, _gpuMaskImg);
  // gpuStream.waitForCompletion();

  // cv::Mat colorMask;
  _gpuColorImg.download(color);
  _gpuMaskImg.download(mask);
  // _gpuMaskImg.release();
  // _gpuColorImg.release();
  ROS_DEBUG_STREAM_THROTTLE(0.5, "first: " << (ros::Time::now() - timer).toSec() * 1000.0);

  // mask = colorMask.clone();

  timer = ros::Time::now();
  cv::findContours(mask, _cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (_cnts.size() > 0) {
    uint8_t index = 0;
    uint8_t bestIdx = 0;
    std::vector<cv::Point> *bestCnt = &_cnts[0];
    size_t maxSiza = _cnts[0].size();
    for (auto &cnt : _cnts) {
      ++index;
      if (cnt.size() < 20) ///< TODO: to params
        continue;

      if (cnt.size() > maxSiza) {
        bestCnt = &cnt;
        maxSiza = cnt.size();
        bestIdx = index - 1;
      }
    }

    float circleRadius = 0.0;
    cv::Point2f circleCenter;
    cv::minEnclosingCircle(*bestCnt, circleCenter, circleRadius);
    cv::drawContours(mask, _cnts, bestIdx, cv::Scalar::all(255), cv::LineTypes::FILLED);

    int offset = 7;
    cv::Rect roi(cv::Point(circleCenter.x - (circleRadius + offset), circleCenter.y - (circleRadius + offset)), cv::Size(circleRadius + circleRadius + offset, circleRadius + circleRadius + offset));
    cv::Mat maskRoi(mask, roi);

    cv::erode(maskRoi, maskRoi, _closeFilterKernel, cv::Point(), 2);
    cv::dilate(maskRoi, maskRoi, _closeFilterKernel, cv::Point(), 5);

    if (center != nullptr)
      *center = circleCenter;

    if (radius != nullptr)
      *radius = static_cast<uint16_t>(std::round(circleRadius));

    /* cv::circle(frame, center, radius + 7.0, cv::Scalar(_threshold[1]), 1, cv::LINE_4);
    cv::circle(frame, center, 3, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
    cv::Point2f textPos = {center.x + radius + 15.0f, center.y + radius + 15.0f};
    cv::putText(frame, _info.str(), textPos, cv::FONT_HERSHEY_SIMPLEX, .6, cv::Scalar::all(0), 2); */
    // mask = colorMask.clone();
    _cnts.clear();
    ROS_DEBUG_STREAM_THROTTLE(0.5, "second: " << (ros::Time::now() - timer).toSec() * 1000.0);
  }
}
