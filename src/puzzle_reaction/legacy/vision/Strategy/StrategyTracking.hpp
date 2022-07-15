#if !defined(_STRATEGY_TRACKING_H_)
#define _STRATEGY_TRACKING_H_

#include "../utils/GyverFilters/src/filters/median3.h"
#include "BallTracking.hpp"
#include "Strategy.hpp"
#include "VideoHandler.hpp"
#include "ros/ros.h"
#include "utils/Utils.hpp"
#include "utils/thresholdtype.hpp"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/highgui/highgui.hpp>
#include <queue>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_listener.h>

using geometry_msgs::TransformStamped;
using image_geometry::PinholeCameraModel;

// class BallTrackingRos;

class StrategyTracking : public Strategy {
private:
  const char *_winName = "Tracking";
  float _filterGain = 0.65f;
  int _fps = 0;

  BallTracking _bt;
  VideoHandler &_vh;
  cv::Mat _frame;
  cv::Mat _depth;
  cv::Point3d _ballPos;
  // BallTrackingRos *_context;
  ros::Time _timer;
  ros::Time _lastDetection;
  PinholeCameraModel _cameraModel;
  tf2_ros::Buffer _tfBuffer;
  tf2_ros::TransformListener _tfListener;
  std::list<cv::Point3d> _ballTragectory;
  std::list<geometry_msgs::Point> _ballPredictedTraj;
  std::list<std::list<geometry_msgs::Point>> _ballPredictedTrajs;
  cv::Point3d startPoint;
  ros::Time startTime;
  ros::Time resetTimer;

  bool _isFirstDetection = true;

public:
  StrategyTracking(VideoHandler &vh, BallTrackingRos *context) : _vh(vh), _context(context), _tfListener(_tfBuffer) {
    cv::namedWindow(_winName, cv::WINDOW_AUTOSIZE);
    cv::namedWindow("test", cv::WINDOW_AUTOSIZE);
    _timer = ros::Time::now();
    _lastDetection = ros::Time::now();
    resetTimer = ros::Time::now();
  }

  ~StrategyTracking() {
    std::cout << "Delete strategy\n";
    cv::destroyWindow(_winName);
    cv::destroyWindow("test");
  }
  GMedian3<double> _medianFilter;
  bool init();
  void execute() override;
  // void fixTragectory(std::list<std::list<geometry_msgs::Point>> &trgs);

  inline void setFilterGain(float gain) { (gain < 0 ? _filterGain = abs(gain) : _filterGain = gain); };
};

#endif // _STRATEGY_TRACKING_H_
