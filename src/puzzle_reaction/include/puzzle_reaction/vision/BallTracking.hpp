#if !defined(_BALL_TRACKING_H_)
#define _BALL_TRACKING_H_

#include "utils/thresholdtype.hpp"
#include <ros/ros.h>
#include <deque>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>


class BallTracking {

private:
  cv::Size2i _imSize;
  threshold_t _threshold;
  std::vector<std::vector<cv::Point2i>> _cnts;
  // std::stringstream _info;

  cv::Size2i _filterSize = cv::Size2i(3, 3);
  cv::Matx33f _dist2Meters = cv::Matx33d::all(0.001f);

  cv::cuda::GpuMat _gpuMaskImg, _gpuColorImg;
  cv::Ptr<cv::cuda::Filter> _gaussFilter;
  cv::Ptr<cv::cuda::Filter> _closeFilter;
  cv::Ptr<cv::cuda::Filter> _erodeFilter;
  cv::Ptr<cv::cuda::Filter> _dilateFilter;
  cv::Mat _closeFilterKernel;

  // static const char *_confFile;

public:
  BallTracking(uint16_t width, uint16_t height, cv::Vec<cv::Scalar_<uint8_t>, 2> threshold);
  ~BallTracking() {}

  // void operator=(const BallTracking &bt);
  /**
   * Handle image and search target color
   * @note not modify color and mask parametrs
   * @param[in,out] color color image
   * @param[out] mask binary area with detected color
   * @param[out] center center of target color area
   * @param[out] radius area radius
   */
  void process(cv::Mat &color, cv::Mat &mask, cv::Point2f *center = nullptr, uint16_t *radius = nullptr);

  inline void setThreshold(const threshold_t th) { _threshold = th; }
};

#endif // _BALL_TRACKING_H_
