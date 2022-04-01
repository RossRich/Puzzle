#include "../../include/PxSitl/vision/RosVH.hpp"

RosVH::RosVH(ros::NodeHandle &nh, image_transport::ImageTransport &it, uint16_t width, uint16_t height)
    : VideoHandler(width, height), _nh(nh), _it(it) {

  if (!loadParam()) {
    ROS_ERROR("Parameters not loaded.");
    return;
  }

  _itRgb.subscribe(_it, _colorTopic, 1);
  _itDepth.subscribe(_it, _depthTopic, 1);
  _sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(3), _itRgb, _itDepth);
  _sync->registerCallback(boost::bind(&RosVH::imageSubCb, this, _1, _2));
}

RosVH::~RosVH() { delete _sync; }

bool RosVH::loadParam() {
  if (!_nh.getParam("color_topic", _colorTopic)) {
    ROS_ERROR("No color_topic param");
    return false;
  } else {
    ROS_INFO("RGB image stream from %s", _colorTopic.c_str());
  }

  if (!_nh.getParam("depth_topic", _depthTopic)) {
    ROS_ERROR("No depth_topic param");
    return false;
  } else {
    ROS_INFO("Depth stream from %s", _depthTopic.c_str());
  }

  return true;
}

void RosVH::readColor(cv::Mat &colorFrame) {
  if (_color)
    colorFrame = _color->image;
}

void RosVH::readDepth(cv::Mat &depthFrame) {
  if (_depth)
    depthFrame = _depth->image;
}

void RosVH::readFrameset(cv::Mat &colorFrame, cv::Mat &depthFrame) {
  readColor(colorFrame);
  readDepth(depthFrame);
}

void RosVH::imageSubCb(const ImageConstPtr &rgb, const ImageConstPtr &d) {

  try {
    _color = cv_bridge::toCvCopy(rgb, enc::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert  from '%s' to 'bgr8'.", _color->encoding.c_str());
  }

  try {
    _depth = cv_bridge::toCvCopy(d, enc::TYPE_16UC1);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert  from '%s' to 'TYPE_16UC1'", _depth->encoding.c_str());
  }

  setLastTime(_color->header.stamp.toSec());
}