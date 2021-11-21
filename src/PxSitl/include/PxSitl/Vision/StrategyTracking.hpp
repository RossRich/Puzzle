#if !defined(_STRATEGY_TRACKING_H_)
#define _STRATEGY_TRACKING_H_

#include <opencv2/highgui/highgui.hpp>
#include "BallTracking.hpp"
#include "Strategy.hpp"
#include "VideoHandler.hpp"
#include "Utils/thresholdtype.hpp"
#include "Utils/Utils.hpp"
#include "ros/ros.h"
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>

using geometry_msgs::TransformStamped;
using image_geometry::PinholeCameraModel;

class BallTrackingRos;

class StrategyTracking : public Strategy {
private:
  const char* _winName = "Tracking";
  BallTracking _bt;
  VideoHandler &_vh;
  cv::Mat _frame;
  cv::Mat _depth;
  BallTrackingRos *_context;
  ros::Time _timer;
  ros::Duration _timeOut = ros::Duration(1);
  PinholeCameraModel _cameraModel;
  tf2_ros::Buffer _tfBuffer;
  tf2_ros::TransformListener _tfListener;
  

public:
  StrategyTracking(VideoHandler &vh, BallTrackingRos *context) : _vh(vh), _context(context), _tfListener(_tfBuffer) {
    cv::namedWindow(_winName, cv::WINDOW_AUTOSIZE);
    cv::namedWindow("test", cv::WINDOW_AUTOSIZE);
    _timer = ros::Time::now();
  }

  ~StrategyTracking() {
    std::cout << "Delete strategy\n";
    cv::destroyWindow(_winName);
    cv::destroyWindow("test");
  }

  bool init();
  void execute() override;
};

#endif // _STRATEGY_TRACKING_H_
