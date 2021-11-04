#if !defined(_BALL_TRACKING_ROS_H_)
#define _BALL_TRACKING_ROS_H_

#include "VideoHandler.hpp"
#include "BallTracking.hpp"
#include "TrackingParam.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

class BallTrackingRos {
  
private:
  ros::NodeHandle &_nh;
  VideoHandler &_vh;
  BallTracking _bt;

  void updateDetector();

public:
  BallTrackingRos(ros::NodeHandle &nh, VideoHandler &vh);
  ~BallTrackingRos();

  bool loadParam();
  void tracking();
};

#endif // _BALL_TRACKING_ROS_H_
