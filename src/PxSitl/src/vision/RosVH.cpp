#include "../../include/PxSitl/Vision/RosVH.hpp"

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

RosVH::~RosVH() {
    delete _sync;
}

bool RosVH::loadParam() {

  if (!_nh.getParam("color_topic", _colorTopic)) {
    ROS_ERROR("No color_topic param");
  }

  if (!_nh.getParam("depth_topic", _depthTopic)) {
    ROS_ERROR("No depth_topic param");
  }

  return true;
}

void RosVH::read(cv::Mat &frame) { frame = _color->image; }

void RosVH::imageSubCb(const ImageConstPtr &rgb, const ImageConstPtr &d) {

  try {
    _color = cv_bridge::toCvCopy(rgb, enc::RGB8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert  from '%s' to 'bgr8'.", _color->encoding.c_str());
  }

  try {
    _depth = cv_bridge::toCvCopy(d, enc::TYPE_16UC1);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert  from '%s' to 'TYPE_16UC1'", _depth->encoding.c_str());
  }
}