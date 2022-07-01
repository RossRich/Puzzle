#include "PxSitl/vision/RosVH.hpp"

RosVH::RosVH(ros::NodeHandle &nh, image_transport::ImageTransport &it, uint16_t width, uint16_t height)
    : VideoHandler(width, height), _nh(&nh), _it(&it) {

  if (!loadParam())
    throw ros::Exception("Invalid parameters");

  _itRgb.subscribe(*_it, _colorTopic, 1);
  _itDepth.subscribe(*_it, _depthTopic, 1);

  _sync = std::unique_ptr<message_filters::Synchronizer<MySyncPolicy>>(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(3), _itRgb, _itDepth));
  _sync->registerCallback(boost::bind(&RosVH::imageSubCb, this, _1, _2));
}

bool RosVH::loadParam() {
  if (!_nh->getParam("camera_color_topic", _colorTopic)) {
    ROS_ERROR("[RosVH] No camera_color_topic param");
    return false;
  } else {
    ROS_INFO("[RosVH] RGB image stream from %s", _colorTopic.c_str());
  }

  if (!_nh->getParam("camera_depth_topic", _depthTopic)) {
    ROS_ERROR("[RosVH] Depth topic not define");
    return false;
  } else {
    ROS_INFO("[RosVH] Depth stream from %s", _depthTopic.c_str());
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
    ROS_ERROR("[RosVH] Could not convert  from '%s' to 'bgr8'.", _color->encoding.c_str());
    ros::Duration(3.0).sleep();
  }

  try {
    _depth = cv_bridge::toCvCopy(d, enc::TYPE_16UC1);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("[RosVH] Could not convert  from '%s' to 'TYPE_16UC1'", _depth->encoding.c_str());
    ros::Duration(3.0).sleep();
  }

  setLastTime(_color->header.stamp.toSec());
}

RosVH &RosVH::operator=(RosVH &&vh) {
  if (this == &vh)
    return *this;

  VideoHandler::operator=(vh);

  _nh = vh._nh;
  _it = vh._it;
  vh._nh = nullptr;
  vh._it = nullptr;

  _colorTopic = vh._colorTopic;
  _depthTopic = vh._depthTopic;

  vh._itDepth.unsubscribe();
  vh._itRgb.unsubscribe();
  _itRgb.subscribe(*_it, _colorTopic, 1);
  _itDepth.subscribe(*_it, _depthTopic, 1);

  vh._sync.release();
  _sync = std::unique_ptr<message_filters::Synchronizer<MySyncPolicy>>(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(3), _itRgb, _itDepth));
  _sync->registerCallback(boost::bind(&RosVH::imageSubCb, this, _1, _2));
  return *this;
}