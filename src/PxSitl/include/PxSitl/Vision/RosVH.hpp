#if !defined(_ROS_VIDEO_HANDLER_H_)
#define _ROS_VIDEO_HANDLER_H_

#include "VideoHandler.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>

using sensor_msgs::ImageConstPtr;
using image_transport::ImageTransport;
using image_transport::SubscriberFilter;
using std::string;

namespace enc = sensor_msgs::image_encodings;

class RosVH : public VideoHandler {
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

private:
  string _colorTopic;
  string _depthTopic;

  ros::NodeHandle &_nh;
  ImageTransport &_it;
  SubscriberFilter _itRgb;
  SubscriberFilter _itDepth;
  message_filters::Synchronizer<MySyncPolicy> *_sync;
  cv_bridge::CvImagePtr _color;
  cv_bridge::CvImagePtr _depth;
  void imageSubCb(const ImageConstPtr &rgb, const ImageConstPtr &depth);
  bool loadParam();

public:
  RosVH(ros::NodeHandle &nh, ImageTransport &it, uint16_t width, uint16_t height);
  ~RosVH();

  void readColor(cv::Mat &colorFrame) override;
  void readDepth(cv::Mat &depthFrame) override;
  void readFrameset(cv::Mat &colorFrame, cv::Mat &depthFrame) override; 
};

#endif // _ROS_VIDEO_HANDLER_H_
