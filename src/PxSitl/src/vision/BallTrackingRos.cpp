#include "../../include/PxSitl/vision/BallTrackingRos.hpp"

BallTrackingRos *BallTrackingRos::getInstanse(ros::NodeHandle &nh) {

  if (_instance != nullptr)
    return _instance;

  int cameraWidth, cameraHeight = 0;
  ros::param::param<int>("/camera/realsense2_camera/color_width", cameraWidth, 0);
  ros::param::param<int>("/camera/realsense2_camera/color_height", cameraHeight, 0);

  if (cameraHeight <= 0 || cameraWidth <= 0) {
    ROS_INFO("No camera resolution in params. Trying to request resolution from CameraInfo...");
    CameraInfoConstPtr cameraInfo;
    try {
      cameraInfo = ros::topic::waitForMessage<CameraInfo>("/camera/color/camera_info", nh);
    } catch (const ros::Exception &e) {
      ROS_ERROR("%s", e.what());
      ROS_ERROR("Failed to get camera info. Exit.");
      return nullptr;
    }

    if (!cameraInfo) {
      ROS_ERROR("Failed to get camera info. Exit.");
      return nullptr;
    }

    cameraWidth = cameraInfo->width;
    cameraHeight = cameraInfo->height;

    if (cameraHeight <= 0 || cameraWidth <= 0) {
      ROS_ERROR("No camera rosolution. Exit.");
      return nullptr;
    }
  }

  ROS_INFO("Camera resolution received. Width = %i, height = %i", cameraWidth, cameraHeight);

  _instance = new BallTrackingRos(nh);
  return _instance;
}

BallTrackingRos::BallTrackingRos(ros::NodeHandle &nh) : _nh(nh) {

  _strategySrv = _nh.advertiseService("strategy_srv", &BallTrackingRos::runSetupSrv, this);
  ROS_INFO("Change strategy server ready");

  _stateWait = new StateWait(*this);
  _stateTracking = new StateTracking(*this, _nh);

  setState(_stateWait);
}

BallTrackingRos::~BallTrackingRos() {
  delete _stateWait;
  delete _stateTracking;
}

bool BallTrackingRos::runSetupSrv(std_srvs::EmptyRequest &request, std_srvs::EmptyResponse &response) {
  return true;
}

void BallTrackingRos::shutdown() {
  _nh.shutdown();
  // ros::shutdown();
}

void BallTrackingRos::loop() {
  _state->execute();
}

void BallTrackingRos::setState(State *state) {
  _state = state;
}