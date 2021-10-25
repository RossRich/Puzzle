#if !defined(_BALL_TRACKING_ROS_H_)
#define _BALL_TRACKING_ROS_H_

#include "BallTracking.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

using sensor_msgs::ImageConstPtr;

namespace enc = sensor_msgs::image_encodings;

class BallTrackingRos {
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
private:
  ros::NodeHandle _nh;
  image_transport::ImageTransport _it;
  image_transport::SubscriberFilter _itRgb;
  image_transport::SubscriberFilter _itDepth;
  message_filters::Synchronizer<MySyncPolicy> *_sync;
  BallTracking _bt;
  cv_bridge::CvImagePtr _color;
  cv_bridge::CvImagePtr _depth;
  const char *_winName = "Main";
  void imageSubCb(const ImageConstPtr &rgb, const ImageConstPtr &depth);

public:
  BallTrackingRos(const ros::NodeHandle &nh, const image_transport::ImageTransport &it);
  ~BallTrackingRos();
  void run();
};

#endif // _BALL_TRACKING_ROS_H_
